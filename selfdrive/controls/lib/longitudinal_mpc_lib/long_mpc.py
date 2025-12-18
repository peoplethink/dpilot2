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
XState = log.LongitudinalPlan.XState

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

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 150.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
ACCEL_MIN = -4.0
ACCEL_MAX = 2.5
T_FOLLOW = 1.25
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0


# === KRKeegan 스타일의 stopped_equivalence + 기본 버전 겸용 ===
def get_stopped_equivalence_factor(v_lead, v_ego, t_follow=T_FOLLOW,
                                   stop_distance=STOP_DISTANCE,
                                   comfort_brake=COMFORT_BRAKE,
                                   krkeegan=False):
  if not krkeegan:
    return (v_lead**2) / (2 * comfort_brake)

  v_lead = np.array(v_lead)
  v_ego = np.array(v_ego)

  v_diff = v_lead - v_ego
  v_diff_offset = 0.0

  if np.all(v_diff > 0):
    v_diff_offset = v_diff * 1.0
    v_diff_offset = np.clip(v_diff_offset, 0.0, stop_distance / 2.0)
    v_diff_offset = np.maximum(v_diff_offset * ((10.0 - v_ego) / 10.0), 0.0)

  distance = (v_lead**2) / (2.0 * comfort_brake) + v_diff_offset
  return distance


def get_safe_obstacle_distance(v_ego, t_follow, comfort_brake, stop_distance):
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance


def desired_follow_distance(v_ego, v_lead, t_follow,
                            stop_distance=STOP_DISTANCE,
                            comfort_brake=COMFORT_BRAKE):
  return (get_safe_obstacle_distance(v_ego, t_follow, comfort_brake, stop_distance)
          - get_stopped_equivalence_factor(v_lead, v_ego, t_follow,
                                           stop_distance, comfort_brake,
                                           krkeegan=False))


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

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

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow,
                                                    comfort_brake, stop_distance)

  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
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
                         lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0,
                                   T_FOLLOW, LEAD_DANGER_FACTOR,
                                   COMFORT_BRAKE, STOP_DISTANCE])

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

    # --- tFollow / Gap 관련 (요청 수정 반영) ---
    self.openpilotLongitudinalControl = False
    self.mySafeModeFactor = 1.0
    self.applyCruiseGap = 1
    self.tFollowSpeedRatio = 1.2
    self.tFollowGap1 = 1.1
    self.tFollowGap2 = 1.2
    self.tFollowGap3 = 1.4
    self.tFollowGap4 = 1.6
    self.v_ego_kph_prev = 0.0

    # dynamic cost
    self.applyLongDynamicCost = False
    self.t_follow = T_FOLLOW

    # timers / debug
    self.lo_timer = 0

    self.reset()
    self.source = SOURCES[2]

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
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      W[4, 4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0],
                                            [1.0, 1.0, 0.0])
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
      base_a_change_cost = A_CHANGE_COST if prev_accel_constraint else 40.0
      if self.applyLongDynamicCost:
        a_mul, j_mul, d_zone_tf = self.get_cost_multipliers(v_lead0, v_lead1)
        cost_weights = [
          X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST,
          base_a_change_cost * a_mul,
          J_EGO_COST * j_mul,
        ]
        constraint_cost_weights = [
          LIMIT_COST, LIMIT_COST, LIMIT_COST,
          DANGER_ZONE_COST * d_zone_tf,
        ]
      else:
        cost_weights = [
          X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST,
          base_a_change_cost, J_EGO_COST,
        ]
        constraint_cost_weights = [
          LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST,
        ]

    elif self.mode == 'blended':
      a_change_cost = 50.0 if prev_accel_constraint else 0.0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')

    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj),
                          0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

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

    # min_x_lead: ACCEL_MIN은 ntune에서 뽑아오므로 update에서 반영(여긴 방어적으로 -4)
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (4.0 * 2.0)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def set_accel_limits(self, min_a, max_a):
    self.cruise_min_a = min_a
    self.max_a = max_a

  # ---------- 요청 핵심: tFollow 관련 값/로직을 “첫 코드 방식”으로 이식 ----------
  def update_params(self):
    self.lo_timer += 1
    if self.lo_timer > 200:
      self.lo_timer = 0
    elif self.lo_timer == 60:
      self.applyLongDynamicCost = Params().get_bool("ApplyLongDynamicCost")
    elif self.lo_timer == 80:
      # mySafeModeFactor는 controls.mySafeModeFactor가 우선이지만, fallback으로 Params도 유지
      # (Params에 있다면 읽되, update()에서 controls로 덮어씁니다)
      try:
        v = self.params_reader.get("MySafeModeFactor", encoding="utf8")
        if v is not None:
          self.mySafeModeFactor = float(int(v)) / 100.0
      except Exception:
        pass
      self.mySafeModeFactor = clip(float(self.mySafeModeFactor), 0.5, 1.0)
    elif self.lo_timer == 100:
      try:
        self.tFollowSpeedRatio = float(int(Params().get("TFollowSpeedRatio", encoding="utf8"))) / 100.
        self.tFollowGap1 = float(int(Params().get("TFollowGap1", encoding="utf8"))) / 100.
        self.tFollowGap2 = float(int(Params().get("TFollowGap2", encoding="utf8"))) / 100.
        self.tFollowGap3 = float(int(Params().get("TFollowGap3", encoding="utf8"))) / 100.
        self.tFollowGap4 = float(int(Params().get("TFollowGap4", encoding="utf8"))) / 100.
      except Exception:
        pass

  def update_gap_tf(self, controls, v_ego, a_ego):
    """
    첫 코드 update_gap_tf(controls, v_ego, a_ego) 로직을 그대로 반영.
    - applyCruiseGap = clip(controls.longCruiseGap, 1, 4)
    - openpilotLongitudinalControl=True: 감속(속도 감소) 시 t_follow 계산 스킵
    - openpilotLongitudinalControl=False: lead 있으면 감속 a_ego로 gap 자동확대(최대 4)
    """
    v_ego_kph = v_ego * CV.MS_TO_KPH

    # 정식 필드명: controls.mySafeModeFactor, controls.longCruiseGap
    self.mySafeModeFactor = clip(float(getattr(controls, "mySafeModeFactor", 1.0)), 0.5, 1.0)

    # gap 입력(정식필드): controls.longCruiseGap
    self.applyCruiseGap = int(clip(int(getattr(controls, "longCruiseGap", 1)), 1, 4))

    if self.openpilotLongitudinalControl:
      if v_ego_kph >= self.v_ego_kph_prev:  # 감속일때는 t_follow(gap) 계산안함
        cruiseGap_dict = {
          1: self.tFollowGap1,
          2: self.tFollowGap2,
          3: self.tFollowGap3,
          4: self.tFollowGap4,
        }
        tf = cruiseGap_dict[self.applyCruiseGap]
        cruiseGapRatio = interp(v_ego_kph, [0, 100], [tf, tf * self.tFollowSpeedRatio])
        self.t_follow = max(0.6, cruiseGapRatio * (2.0 - self.mySafeModeFactor))
    else:
      if self.status:
        if v_ego_kph < 0.1:
          self.applyCruiseGap = 1
        else:
          # 감속이 큰 경우 gap을 키움(최대 4)
          self.applyCruiseGap = int(interp(a_ego, [-1.5, -0.5], [4, self.applyCruiseGap]))

      # openpilotLongitudinalControl=False에서도 t_follow는 gap 기반으로 만들어주는 게 안정적이라 추가(첫 코드 흐름과 동일한 결과)
      cruiseGap_dict = {
        1: self.tFollowGap1,
        2: self.tFollowGap2,
        3: self.tFollowGap3,
        4: self.tFollowGap4,
      }
      tf = cruiseGap_dict[int(clip(self.applyCruiseGap, 1, 4))]
      cruiseGapRatio = interp(v_ego_kph, [0, 100], [tf, tf * self.tFollowSpeedRatio])
      self.t_follow = max(0.6, cruiseGapRatio * (2.0 - self.mySafeModeFactor))

    self.v_ego_kph_prev = v_ego_kph
  # -------------------------------------------------------------------
  def update(self, carstate, radarstate, model, controls, v_cruise, x, v, a, j, prev_accel_constraint, reset_state):
    v_ego = self.x0[1]
    a_ego = carstate.aEgo
    
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status
    self.update_params()

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    self.update_gap_tf(controls, v_ego, a_ego)
    self.params[:, 4] = self.t_follow

    # comfort / stopDistance는 ntune 값 + safe factor 적용
    stop_distance = float(ntune_scc_get('stopDistance')) if ntune_scc_get('stopDistance') is not None else STOP_DISTANCE
    comfort_brake = float(ntune_scc_get('comfortBrake')) if ntune_scc_get('comfortBrake') is not None else COMFORT_BRAKE

    applyStopDistance = stop_distance * (2.0 - self.mySafeModeFactor)
    comfort_brake_eff = comfort_brake * self.mySafeModeFactor

    self.params[:, 6] = comfort_brake_eff
    self.params[:, 7] = applyStopDistance

    # accel limits (기존 구조 유지)
    self.params[:,0] = ACCEL_MIN if not reset_state else a_ego
    self.params[:,1] = self.max_a if not reset_state else a_ego

    # stopped equivalence (KRKeegan 옵션 포함)
    lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor(
      lead_xv_0[:, 1],
      self.x_sol[:, 1],
      t_follow=self.t_follow,
      stop_distance=applyStopDistance,
      comfort_brake=comfort_brake_eff,
      krkeegan=self.applyLongDynamicCost
    )
    lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor(
      lead_xv_1[:, 1],
      self.x_sol[:, 1],
      t_follow=self.t_follow,
      stop_distance=applyStopDistance,
      comfort_brake=comfort_brake_eff,
      krkeegan=self.applyLongDynamicCost
    )

    if self.mode == 'acc':
      self.params[:, 5] = LEAD_DANGER_FACTOR

      v_lower = v_ego + (T_IDXS * float(self.params[0, 0]) * 1.05)
      v_upper = v_ego + (T_IDXS * float(self.params[0, 1]) * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)

      cruise_obstacle = (np.cumsum(T_DIFFS * v_cruise_clipped) +
                         get_safe_obstacle_distance(v_cruise_clipped,
                                                    self.t_follow,
                                                    comfort_brake_eff,
                                                    applyStopDistance))

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[int(np.argmin(x_obstacles[0]))]

      # ACC mode는 목표 traj를 0으로(기존 구조 유지)
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:, 5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1, 0] < x_and_cruise[1, 1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    # yref 설정
    self.yref[:, 1] = x
    self.yref[:, 2] = v
    self.yref[:, 3] = a
    self.yref[:, 5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    # solver params 설정
    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.prev_a)
    self.params[:, 4] = self.t_follow  # 최종 확정

    self.run()

    if (np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE) and
        radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

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
