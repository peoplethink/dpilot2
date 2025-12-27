#!/usr/bin/env python3
import os
import numpy as np

from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import clip, interp
from selfdrive.swaglog import cloudlog
from selfdrive.modeld.constants import index_function
from selfdrive.controls.radard import _LEAD_ACCEL_TAU
from selfdrive.controls.ntune import ntune_scc_get

from common.conversions import Conversions as CV
from common.params import Params
from common.filter_simple import StreamingMovingAverage
from cereal import log, car

EventName = car.CarEvent.EventName
XState = log.LongitudinalPlan.XState  # ✅ XState 이식

if __name__ == '__main__':  # generating code
  from pyextra.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython  # pylint: disable=no-name-in-module, import-error

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 8
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

# ====== Costs / constraints ======
X_EGO_OBSTACLE_COST = 6.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0

A_CHANGE_COST = 200.
A_CHANGE_COST_STARTING = 180.0

DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.8
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# ====== Horizon ======
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]
T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])

# ====== Limits / constants ======
ACCEL_MIN = -4.0
ACCEL_MAX = 2.0
T_FOLLOW = 1.25
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.5


# =============================================================================
# numpy runtime helpers (OK with np arrays)
# =============================================================================
def get_safe_obstacle_distance_np(v_ego, t_follow, comfort_brake, stop_distance):
  v_ego = np.asarray(v_ego)
  return (v_ego**2) / (2.0 * comfort_brake) + t_follow * v_ego + stop_distance


def get_stopped_equivalence_factor_np(v_lead, v_ego, t_follow,
                                      stop_distance, comfort_brake,
                                      krkeegan=False):
  if not krkeegan:
    v_lead = np.asarray(v_lead)
    return (v_lead**2) / (2.0 * comfort_brake)

  v_lead = np.asarray(v_lead)
  v_ego  = np.asarray(v_ego)

  v_diff = v_lead - v_ego
  v_diff_offset = 0.0
  if np.all(v_diff > 0):
    v_diff_offset = np.clip(v_diff * 1.0, 0.0, stop_distance / 2.0)
    v_diff_offset = np.maximum(v_diff_offset * ((10.0 - v_ego) / 10.0), 0.0)

  return (v_lead**2) / (2.0 * comfort_brake) + v_diff_offset


# =============================================================================
# CasADi SX helpers (MUST NOT use numpy here)
# =============================================================================
def get_safe_obstacle_distance_sx(v_ego, t_follow, comfort_brake, stop_distance):
  return (v_ego*v_ego) / (2.0 * comfort_brake) + t_follow * v_ego + stop_distance


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # states
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # params
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  comfort_brake = SX.sym('comfort_brake')
  stop_distance = SX.sym('stop_distance')

  model.p = vertcat(a_min, a_max, x_obstacle, prev_a,
                    lead_t_follow, lead_danger_factor,
                    comfort_brake, stop_distance)

  # dynamics
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]
  ocp.dims.N = N

  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))
  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  comfort_brake = ocp.model.p[6]
  stop_distance = ocp.model.p[7]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance_sx(v_ego, lead_t_follow,
                                                      comfort_brake, stop_distance)

  costs = [((x_obstacle - x_ego) - desired_dist_comfort) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) -
                         lead_danger_factor * desired_dist_comfort) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  ocp.constraints.x0 = np.zeros(X_DIM)
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0,
                                   T_FOLLOW, LEAD_DANGER_FACTOR,
                                   COMFORT_BRAKE, STOP_DISTANCE])

  # slack costs set at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc', dt=DT_MDL):
    self.dt = dt
    self.mode = mode
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)

    self.params_reader = Params()

    # ===== runtime states =====
    self.v_cruise = 0.0
    self.x_obstacle_min = 0.0
    self.source = SOURCES[2]
    self.openpilotLongitudinalControl = False

    self.experimentalMode = False

    # longActiveUser 제거 → enabled 엣지 감지로 대체
    self.prev_enabled = False

    # ===== e2e/traffic/stop related =====
    self.trafficState = 0
    self.trafficStopDistanceAdjust = 0.0
    self.applyLongDynamicCost = False
    self.trafficStopAccel = 1.0
    self.trafficStopModelSpeed = True
    self.trafficStopMode = 1
    self.softHoldMode = 1

    self.stopDistance = STOP_DISTANCE
    self.softHoldTimer = 0
    self.lo_timer = 0

    self.applyCruiseGap = 1.
    self.applyModelDistOrder = 32
    self.trafficStopAdjustRatio = 1.0
    self.fakeCruiseDistance = 0.0
    self.stopDist = 0.0
    self.e2eCruiseCount = 0
    self.mpcEvent = 0
    self.prev_x = 0.0
    self.v_ego_kph_prev = 0.0

    self.tFollowSpeedRatio = 1.2
    self.tFollowGap1 = 1.1
    self.tFollowGap2 = 1.2
    self.tFollowGap3 = 1.4
    self.tFollowGap4 = 1.6

    self.xStopFilter = StreamingMovingAverage(3)
    self.xStopFilter2 = StreamingMovingAverage(15)
    self.vFilter = StreamingMovingAverage(10)

    self.t_follow = T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.xState = XState.cruise
    self.xStop = 0.0
    self.e2ePaused = False
    self.trafficError = False
    self.cruiseButtonCounter = 0

    # safety factor from controls
    self.mySafeModeFactor = 1.0

    self.reset()

  def reset(self):
    self.solver.reset()

    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)

    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.params = np.zeros((N+1, PARAM_DIM))

    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0

    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0

    self.x0 = np.zeros(X_DIM)

    self.t_follow = T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.xState = XState.cruise
    self.startSignCount = 0
    self.stopSignCount = 0

    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      W[4, 4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def get_cost_multipliers(self, v_lead0, v_lead1):
    v_ego = self.x0[1]
    v_ego_bps = [0.0, 10.0]
    TFs = [1.2, 1.45, 1.8]

    a_change_tf = interp(self.t_follow, TFs, [.8, 1., 1.1])
    j_ego_tf = interp(self.t_follow, TFs, [.8, 1., 1.1])
    d_zone_tf = interp(self.t_follow, TFs, [1.3, 1., 1.])

    j_ego_v_ego = 1.0
    a_change_v_ego = 1.0
    if (v_lead0 - v_ego >= 0) and (v_lead1 - v_ego >= 0):
      j_ego_v_ego = interp(v_ego, v_ego_bps, [.05, 1.])
      a_change_v_ego = interp(v_ego, v_ego_bps, [.05, 1.])

    j_ego = min(j_ego_tf, j_ego_v_ego)
    a_change = min(a_change_tf, a_change_v_ego)
    return (a_change, j_ego, d_zone_tf)

  def set_weights(self, prev_accel_constraint=True, v_lead0=0.0, v_lead1=0.0):
    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else A_CHANGE_COST_STARTING
      if self.applyLongDynamicCost:
        a_mul, j_mul, d_zone_tf = self.get_cost_multipliers(v_lead0, v_lead1)
        cost_weights = [
          X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST,
          a_change_cost * a_mul,
          J_EGO_COST * j_mul,
        ]
        constraint_cost_weights = [
          LIMIT_COST, LIMIT_COST, LIMIT_COST,
          DANGER_ZONE_COST * d_zone_tf,
        ]
      else:
        cost_weights = [
          X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST,
          a_change_cost, J_EGO_COST,
        ]
        constraint_cost_weights = [
          LIMIT_COST, LIMIT_COST, LIMIT_COST,
          DANGER_ZONE_COST,
        ]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 40.0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')

    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.0:
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2) / 2.0)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    return np.column_stack((x_lead_traj, v_lead_traj))

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    min_x_lead = ((v_ego + v_lead) / 2.0) * (v_ego - v_lead) / (-ACCEL_MIN * 2.0)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10.0, 5.0)
    return self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)

  def set_accel_limits(self, min_a, max_a):
    self.cruise_min_a = float(min_a)
    self.max_a = float(max_a)

  # =========================
  # Params update (EndToEndLong 추가)
  # =========================
  def update_params(self):
    self.lo_timer += 1
    if self.lo_timer > 200:
      self.lo_timer = 0
    elif self.lo_timer == 40:
      try:
        self.trafficStopDistanceAdjust = float(int(Params().get("TrafficStopDistanceAdjust", encoding="utf8"))) / 100.0
      except Exception:
        pass
    elif self.lo_timer == 60:
      self.applyLongDynamicCost = Params().get_bool("ApplyLongDynamicCost")
      try:
        self.trafficStopAccel = float(int(Params().get("TrafficStopAccel", encoding="utf8"))) / 100.0
      except Exception:
        pass
    elif self.lo_timer == 80:
      self.trafficStopModelSpeed = Params().get_bool("TrafficStopModelSpeed")
      try:
        self.trafficStopMode = int(Params().get("TrafficStopMode", encoding="utf8"))
      except Exception:
        pass
      try:
        self.stopDistance = float(int(Params().get("StopDistance", encoding="utf8"))) / 100.0
      except Exception:
        pass
    elif self.lo_timer == 100:
      try:
        self.tFollowSpeedRatio = float(int(Params().get("TFollowSpeedRatio", encoding="utf8"))) / 100.0
        self.tFollowGap1 = float(int(Params().get("TFollowGap1", encoding="utf8"))) / 100.0
        self.tFollowGap2 = float(int(Params().get("TFollowGap2", encoding="utf8"))) / 100.0
        self.tFollowGap3 = float(int(Params().get("TFollowGap3", encoding="utf8"))) / 100.0
        self.tFollowGap4 = float(int(Params().get("TFollowGap4", encoding="utf8"))) / 100.0
      except Exception:
        pass
    elif self.lo_timer == 140:
      try:
        self.softHoldMode = int(Params().get("SoftHoldMode", encoding="utf8"))
      except Exception:
        pass
    elif self.lo_timer == 160:
      try:
        self.applyModelDistOrder = int(Params().get("ApplyModelDistOrder", encoding="utf8"))
      except Exception:
        pass
      try:
        self.trafficStopAdjustRatio = float(int(Params().get("TrafficStopAdjustRatio", encoding="utf8"))) / 100.0
      except Exception:
        pass

  # =========================
  # Gap -> t_follow
  # =========================
  def update_gap_tf(self, controls, v_ego, a_ego):
    v_ego_kph = v_ego * CV.MS_TO_KPH

    self.applyCruiseGap = int(clip(int(getattr(controls, "longCruiseGap", 1)), 1, 4))
    self.mySafeModeFactor = clip(float(getattr(controls, "mySafeModeFactor", 1.0)), 0.5, 1.0)

    cruiseGap_dict = {1: self.tFollowGap1, 2: self.tFollowGap2, 3: self.tFollowGap3, 4: self.tFollowGap4}
    tf = cruiseGap_dict[int(clip(self.applyCruiseGap, 1, 4))]
    cruiseGapRatio = interp(v_ego_kph, [0, 100], [tf, tf * self.tFollowSpeedRatio])

    if self.openpilotLongitudinalControl:
      if v_ego_kph >= self.v_ego_kph_prev:
        self.t_follow = max(0.6, cruiseGapRatio * (2.0 - self.mySafeModeFactor))
    else:
      if self.status:
        if v_ego_kph < 0.1:
          self.applyCruiseGap = 1
        else:
          self.applyCruiseGap = int(interp(a_ego, [-1.5, -0.5], [4, self.applyCruiseGap]))
      self.t_follow = max(0.6, cruiseGapRatio * (2.0 - self.mySafeModeFactor))

    self.v_ego_kph_prev = v_ego_kph

  # =========================
  # stop dist helpers
  # =========================
  def update_stop_dist(self, stop_x):
    stop_x = self.xStopFilter.process(stop_x, median=True)
    stop_x = self.xStopFilter2.process(stop_x)
    return stop_x

  def check_model_stopping(self, carstate, v, v_ego, model_x, y):
    v_ego_kph = v_ego * CV.MS_TO_KPH
    model_v = self.vFilter.process(v[-1])

    startSign = (model_v > 5.0 or model_v > (v[0] + 2.0))
    self.prev_x = model_x

    if v_ego_kph < 1.0:
      stopSign = model_x < 20.0 and model_v < 10.0
    elif v_ego_kph < 80.0:
      stopSign = (model_x < 120.0 and ((model_v < 3.0) or (model_v < v[0] * 0.7)) and abs(y[-1]) < 5.0)
    else:
      stopSign = False

    self.stopSignCount = self.stopSignCount + 1 if stopSign else 0
    self.startSignCount = self.startSignCount + 1 if (startSign and not stopSign) else 0

    if self.stopSignCount * DT_MDL > 0.0 and carstate.rightBlinker is False:
      self.trafficState = 1
    elif self.startSignCount * DT_MDL > 0.1:
      self.trafficState = 2
    else:
      self.trafficState = 0

  # =======================
  # =========================
  # MPC mode 결정 + XState 상태기계 (알럿팝업 1회 트리거)
  # =========================
  def update_apilot(self, controls, carstate, radarstate, model, v_cruise, mode):
    v_ego = carstate.vEgo
    v_ego_kph = v_ego * CV.MS_TO_KPH
    enabled_now = bool(getattr(controls, "enabled", True))

    x = model.position.x
    y = model.position.y
    v = model.velocity.x

    self.fakeCruiseDistance = 0.0
    radar_detected = bool(radarstate.leadOne.status and getattr(radarstate.leadOne, "radar", False))

    # stop dist (model)
    stop_x = x[self.applyModelDistOrder] if self.applyModelDistOrder < len(x) else x[-1]
    self.xStop = self.update_stop_dist(stop_x)
    stop_x = self.xStop

    # traffic stopping 판단
    self.check_model_stopping(carstate, v, v_ego, x[-1], y)

    cruiseButtonCounterDiff = controls.cruiseButtonCounter - self.cruiseButtonCounter
    if cruiseButtonCounterDiff != 0:
      self.trafficError = False

    if self.e2eCruiseCount > 0:
      self.e2eCruiseCount -= 1

    # 알럿 이벤트는 "새로" 생길 때만 1회 트리거
    # (없으면 스팸처럼 계속 뜰 수 있음)
    if not hasattr(self, "_last_popup_event"):
      self._last_popup_event = 0

    new_event = 0

    # 운전자 개입/해제시 이벤트 클리어
    if carstate.gasPressed or carstate.brakePressed or (not enabled_now):
      new_event = 0

    # SOFT_HOLD
    if carstate.brakePressed and v_ego < 0.1 and self.softHoldMode > 0:
      self.softHoldTimer += 1
      if self.softHoldTimer * DT_MDL >= 0.7:
        self.xState = XState.softHold
        new_event = EventName.autoHold
    else:
      self.softHoldTimer = 0

    # =========================
    # 상태 머신
    # =========================

    # --- softHold ---
    if self.xState == XState.softHold:
      stop_x = 0.0
      self.trafficError = False

      if carstate.gasPressed:
        self.xState = XState.e2eCruisePrepare

      elif self.trafficState == 2:
        self.xState = XState.e2eCruisePrepare
        self.e2eCruiseCount = int(3 / DT_MDL)
        new_event = EventName.trafficSignGreen

      if cruiseButtonCounterDiff > 0:
        if self.trafficState == 1:
          self.xState = XState.e2eStop
        else:
          self.xState = XState.e2eCruise
          new_event = EventName.trafficSignGreen

    # --- 고속모드 / 신호정지 비활성 ---
    elif getattr(controls, "myDrivingMode", 0) == 4 or self.trafficStopMode == 0:
      self.xState = XState.lead if self.status else XState.cruise
      self.trafficState = 0
      self.trafficError = False
      stop_x = 1000.0

    # --- 신호 감속/정지 중 ---
    elif self.xState == XState.e2eStop:
      if carstate.gasPressed:
        self.xState = XState.e2eCruisePrepare
        stop_x = 1000.0
      else:
        if v_ego < 0.1:
          if self.trafficState == 2 and (not self.trafficError or (self.trafficError and cruiseButtonCounterDiff > 0)):
            self.xState = XState.e2eCruisePrepare
            self.e2eCruiseCount = int(3 / DT_MDL)
            new_event = EventName.trafficSignGreen
          else:
            if self.trafficState == 2 and self.trafficError:
              new_event = EventName.trafficSignChanged

            if self.trafficError and cruiseButtonCounterDiff > 0:
              self.trafficError = False
            elif (not self.trafficError) and cruiseButtonCounterDiff < 0:
              self.trafficError = True

            self.stopDist = 0.0
            v_cruise = 0.0
            stop_x = 0.0

        # 레이더 리드가 정지선보다 더 가까우면 리드 추종으로 전환
        elif radar_detected and (radarstate.leadOne.dRel - stop_x) < 2.0:
          self.xState = XState.lead
          stop_x = 1000.0

        # 운전자 크루즈버튼(+)으로 출발 준비
        elif cruiseButtonCounterDiff > 0:
          self.xState = XState.e2eCruisePrepare
          stop_x = 1000.0

        else:
          self.comfort_brake = COMFORT_BRAKE * self.trafficStopAccel

          # engage 에지에서 stopDist 초기화 (기존 longActiveUser 대체)
          if enabled_now and (not self.prev_enabled):
            self.stopDist = 2.0 if self.xStop < 2.0 else self.xStop
          else:
            if self.trafficState == 2:
              # 감속 중 파란불이면 출발 준비
              self.xState = XState.e2eCruisePrepare
              stop_x = 1000.0
            else:
              stop_dist = self.xStop * interp(self.xStop, [0, 100], [1.0, self.trafficStopAdjustRatio])
              if stop_dist > 5.0:
                self.stopDist = stop_dist
              stop_x = 0.0

          self.fakeCruiseDistance = 0.0 if self.stopDist > 10.0 else 10.0

    # --- e2eCruisePrepare (일시정지/출발 준비) ---
    elif self.xState == XState.e2eCruisePrepare:
      if not enabled_now:
        self.xState = XState.e2eCruise

      # 잘못된 출발 신호 대응: 브레이크/(-) 입력 + 잠깐동안 e2eCruiseCount 유지 시 재정지
      elif (carstate.brakePressed or cruiseButtonCounterDiff < 0) and self.e2eCruiseCount > 0:
        self.xState = XState.e2eStop
        self.stopDist = 2.0
        self.trafficError = True

      # 저속 + 신호출발이 아니거나(또는 버튼으로 확인) -> 다시 정지로
      elif v_ego_kph < 2.0 and (self.trafficState != 2 or cruiseButtonCounterDiff > 0):
        self.xState = XState.e2eStop
        self.stopDist = 2.0

      # 충분히 움직이고(stop_x 멀어짐) -> 일반 주행
      elif v_ego_kph > 5.0 and stop_x > 60.0:
        self.xState = XState.e2eCruise

      else:
        self.trafficError = False
        stop_x = 1000.0

    # --- e2eCruise / lead / cruise ---
    else:
      self.trafficError = False

      if self.status:
        self.xState = XState.lead
        stop_x = 1000.0

      elif abs(carstate.steeringAngleDeg) > 5.0:
        pass

      elif self.trafficState == 1 and not carstate.gasPressed:
        self.xState = XState.e2eStop
        new_event = EventName.trafficStopping
        self.stopDist = self.xStop

      else:
        self.xState = XState.e2eCruise
        if carstate.brakePressed and v_ego_kph < 1.0 and self.softHoldMode > 0:
          self.xState = XState.softHold

      # stop 신호가 아니면 stop_x 무효화
      if self.trafficState in [0, 2]:
        stop_x = 1000.0

    # =========================
    # ✅ ACC / E2E(blended) 전환 조건 (첫번째 코드 + leadOne/leadTwo 비전리드 확장)
    # =========================
    if self.trafficStopMode > 0:
      if self.trafficStopMode == 3:
        leads = [getattr(radarstate, "leadOne", None), getattr(radarstate, "leadTwo", None)]
        vision_detected = False
        for ld in leads:
          if ld is None:
            continue
          if bool(getattr(ld, "status", False)) and (not bool(getattr(ld, "radar", False))) and (float(getattr(ld, "dRel", 1e9)) < 90.0):
            vision_detected = True
            break

        mode = 'blended' if (self.xState in [XState.e2eCruisePrepare] or vision_detected) else 'acc'

      elif self.trafficStopMode == 2:
        mode = 'blended' if (self.xState in [XState.e2eCruisePrepare]) else 'acc'

      else:
        if (self.xState == XState.e2eCruisePrepare) or (self.xState == XState.e2eStop and self.stopDist > 40.0):
          mode = 'blended'
        else:
          mode = 'acc'

    # comfort 적용
    self.comfort_brake *= self.mySafeModeFactor

    # =========================
    # ✅ Alerts popup: 새 이벤트만 1회 트리거
    # =========================
    if new_event != 0 and new_event != self._last_popup_event:
      self.mpcEvent = new_event
      self._last_popup_event = new_event
    else:
      # 평상시에는 이벤트 없음(팝업 안 뜸)
      self.mpcEvent = 0
      if new_event == 0:
        self._last_popup_event = 0

    # housekeeping
    self.prev_enabled = enabled_now
    self.cruiseButtonCounter = controls.cruiseButtonCounter

    # stopDist countdown / clamp
    self.stopDist -= (v_ego * DT_MDL)
    if self.stopDist < 0.0:
      self.stopDist = 0.0
    elif stop_x == 1000.0:
      self.stopDist = 0.0
    elif self.stopDist > 0.0:
      stop_dist = v_ego ** 2 / (2.0 * 2.0)
      self.stopDist = self.stopDist if self.stopDist > stop_dist else stop_dist
      stop_x = 0.0

    return v_cruise, stop_x + self.stopDist, mode

  # =========================
  # Main update
  # =========================
  def update(self, carstate, radarstate, model, controls,
             v_cruise, x, v, a, j, prev_accel_constraint, reset_state):

    self.update_params()

    v_ego = self.x0[1]
    a_ego = carstate.aEgo

    self.mySafeModeFactor = clip(float(getattr(controls, "mySafeModeFactor", 1.0)), 0.5, 1.0)

    self.status = radarstate.leadOne.status or radarstate.leadTwo.status
    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    self.update_gap_tf(controls, v_ego, a_ego)

    self.comfort_brake = COMFORT_BRAKE

    v_cruise, stop_x, self.mode = self.update_apilot(controls, carstate, radarstate, model, v_cruise, self.mode)
    self.mode = 'blended' if self.experimentalMode else self.mode
    

    v_lead0 = lead_xv_0[0, 1]
    v_lead1 = lead_xv_1[0, 1]
    self.set_weights(prev_accel_constraint=prev_accel_constraint, v_lead0=v_lead0, v_lead1=v_lead1)

    stop_distance = float(ntune_scc_get('stopDistance')) if ntune_scc_get('stopDistance') is not None else self.stopDistance
    comfort_brake = float(ntune_scc_get('comfortBrake')) if ntune_scc_get('comfortBrake') is not None else self.comfort_brake

    applyStopDistance = stop_distance * (2.0 - self.mySafeModeFactor)
    comfort_brake_eff = comfort_brake * self.mySafeModeFactor

    self.params[:, 0] = ACCEL_MIN if not reset_state else a_ego
    self.params[:, 1] = self.max_a if not reset_state else a_ego

    lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor_np(
      lead_xv_0[:, 1], self.x_sol[:, 1],
      t_follow=self.t_follow,
      stop_distance=applyStopDistance,
      comfort_brake=comfort_brake_eff,
      krkeegan=self.applyLongDynamicCost
    )
    lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor_np(
      lead_xv_1[:, 1], self.x_sol[:, 1],
      t_follow=self.t_follow,
      stop_distance=applyStopDistance,
      comfort_brake=comfort_brake_eff,
      krkeegan=self.applyLongDynamicCost
    )

    if self.mode == 'acc':
      self.params[:, 5] = LEAD_DANGER_FACTOR

      x2 = stop_x * np.ones(N+1) + self.trafficStopDistanceAdjust

      v_lower = v_ego + (T_IDXS * float(self.params[0, 0]) * 1.05)
      v_upper = v_ego + (T_IDXS * float(self.params[0, 1]) * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)

      cruise_obstacle = (np.cumsum(T_DIFFS * v_cruise_clipped) +
                         get_safe_obstacle_distance_np(v_cruise_clipped,
                                                       self.t_follow,
                                                       comfort_brake_eff,
                                                       applyStopDistance + self.fakeCruiseDistance))

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, x2])
      self.source = SOURCES[int(np.argmin(x_obstacles[0]))]

      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:, 5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])

      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2.0) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1, 0] < x_and_cruise[1, 1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:, 1] = x
    self.yref[:, 2] = v
    self.yref[:, 3] = a
    self.yref[:, 5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.prev_a)
    self.params[:, 4] = self.t_follow
    self.params[:, 6] = comfort_brake_eff
    self.params[:, 7] = applyStopDistance

    self.run()

    if (np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE) and
        radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    self.v_cruise = v_cruise
    self.x_obstacle_min = self.params[:, 2]

  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:, 1]
    self.a_solution = self.x_sol[:, 2]
    self.j_solution = self.u_sol[:, 0]

    self.prev_a = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = sec_since_boot()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
