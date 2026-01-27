#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import clip, interp

import cereal.messaging as messaging
from cereal import log

from common.conversions import Conversions as CV
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N, ACCEL_MIN, ACCEL_MAX
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.vision_turn_controller import VisionTurnController
from common.params import Params
from selfdrive.controls.lib.events import Events

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.5, 1.3, 0.4, 0.2, 0.15, 0.1]
A_CRUISE_MAX_BP = [
  0.,
  30 * CV.KPH_TO_MS,
  50 * CV.KPH_TO_MS,
  70 * CV.KPH_TO_MS,
  90 * CV.KPH_TO_MS,
  110 * CV.KPH_TO_MS
]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.dt = dt
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.v_model_error = 0.0

    self.x_desired_trajectory = np.zeros(CONTROL_N)
    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.params = Params()
    self.param_read_counter = 0

    self.vCluRatio = 1.0

    self.myEcoModeFactor = 1.0
    self.myDrivingMode = 3  # ✅ UI(Params) 기반으로만 사용할 driving mode (기본: 일반)

    # ✅ A안: 부팅 1회 InitMyDrivingMode -> MyDrivingMode 이관 여부
    self._init_md_applied = False

    self.params_count = 0

    # (초기값) 파라미터 없을 수 있으니 안전하게 기본값 세팅
    self.cruiseMaxVals1 = 1.5
    self.cruiseMaxVals2 = 1.3
    self.cruiseMaxVals3 = 0.4
    self.cruiseMaxVals4 = 0.2
    self.cruiseMaxVals5 = 0.15
    self.cruiseMaxVals6 = 0.1

    # cluster speed 사용 여부
    self.use_cluster_speed = self.params.get_bool('UseClusterSpeed')
    self.cruise_source = 'cruise'
    self.vision_turn_controller = VisionTurnController(CP)
    self.events = Events()

    self.mpc.openpilotLongitudinalControl = CP.openpilotLongitudinalControl

    # initial read
    self.read_param()

  def read_param(self):
    # ✅ 안전하게: 값이 없거나 파싱 실패해도 크래시 안 나게
    try:
      self.myEcoModeFactor = float(int(self.params.get("MyEcoModeFactor", encoding="utf8"))) / 100.
    except Exception:
      pass

    for i in range(1, 7):
      try:
        v = float(int(self.params.get(f"CruiseMaxVals{i}", encoding="utf8"))) / 100.
        setattr(self, f"cruiseMaxVals{i}", v)
      except Exception:
        pass

    # ✅ UI에서만 실시간 반영: controlsState.myDrivingMode 무시하고 Params만 사용
    try:
      v = self.params.get("MyDrivingMode", encoding="utf8")
      if v is not None and len(v):
        self.myDrivingMode = int(v)
    except Exception:
      pass
    self.myDrivingMode = int(clip(int(self.myDrivingMode), 1, 5))

    # ✅ A안: 부팅 1회 InitMyDrivingMode -> MyDrivingMode 이관(Planner가 Params 직접 읽는 구조라서 여기서 보장)
    if not self._init_md_applied:
      try:
        init_md = self.params.get("InitMyDrivingMode", encoding="utf8")
        if init_md is not None and len(init_md):
          init_md_i = int(init_md)
          init_md_i = int(clip(int(init_md_i), 1, 5))
          # LIVE 값으로 복사
          self.params.put("MyDrivingMode", str(init_md_i))
          self.myDrivingMode = init_md_i
      except Exception:
        pass
      self._init_md_applied = True

  def get_max_accel(self, v_ego):
    cruiseMaxVals = [
      self.cruiseMaxVals1, self.cruiseMaxVals2, self.cruiseMaxVals3,
      self.cruiseMaxVals4, self.cruiseMaxVals5, self.cruiseMaxVals6
    ]
    return interp(v_ego, A_CRUISE_MAX_BP, cruiseMaxVals)

  # ✅ FIX: staticmethod 제거 + 인자/호출 일치 (model_msg가 float로 들어가던 크래시 해결)
  def parse_model(self, model_msg, model_error, v_ego):
    # modelV2 메시지가 깨졌을 때(혹은 None) 크래시 방지
    if model_msg is None or not hasattr(model_msg, 'position'):
      x = np.zeros(len(T_IDXS_MPC))
      v = np.ones(len(T_IDXS_MPC)) * float(v_ego)
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
      return x, v, a, j

    if (len(model_msg.position.x) == 33 and
        len(model_msg.velocity.x) == 33 and
        len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    return x, v, a, j

  def update(self, sm):
    # ✅ UI 실시간 반영: 더 자주 읽기(0.1s 정도 체감)
    if self.param_read_counter % 10 == 0:
      self.read_param()
    self.param_read_counter += 1

    self.mpc.experimentalMode = sm['controlsState'].experimentalMode

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['controlsState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    # neokii: cluster ratio 적용 (ported, 안전하게)
    if not self.use_cluster_speed:
      vCluRatio = getattr(sm['carState'], 'vCluRatio', 1.0)
      if vCluRatio > 0.5:
        self.vCluRatio = vCluRatio
        v_cruise *= vCluRatio
        v_cruise = int(v_cruise * CV.MS_TO_KPH + 0.25) * CV.KPH_TO_MS

    mySafeModeFactor = sm['controlsState'].mySafeModeFactor

    # ✅ UI(Params)에서만 실시간 반영되도록 고정
    myDrivingMode = self.myDrivingMode

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      if myDrivingMode in [1]:  # 연비
        myMaxAccel = clip(self.get_max_accel(v_ego) * self.myEcoModeFactor, 0, ACCEL_MAX)
      elif myDrivingMode in [2]:  # 안전
        myMaxAccel = clip(self.get_max_accel(v_ego) * self.myEcoModeFactor * mySafeModeFactor, 0, ACCEL_MAX)
      elif myDrivingMode in [3, 4]:  # 일반, 고속
        myMaxAccel = clip(self.get_max_accel(v_ego), 0, ACCEL_MAX)
      else:
        myMaxAccel = self.get_max_accel(v_ego)

      accel_limits = [A_CRUISE_MIN, myMaxAccel]
      accel_limits_turns = limit_accel_in_turns(
        v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP
      )
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])
      # mpc에서는 prev_a를 참고하여 constraint작동함.... pid off -> on시에는 현재 constraint가 작동하지 않아서 집어넣어봄...
      self.mpc.prev_a = np.full(N + 1, self.a_desired)
      # ✅ FIX: 중복 대입 제거 (오타) + 하한 0으로(원 코드 유지)
      accel_limits_turns[0] = 0.0

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    if force_slow_decel:
      v_cruise = 0.0

    # Get acceleration and active solutions for custom long mpc.
    v_cruise = self.cruise_solutions(
      not reset_state,
      self.v_desired_filter.x,
      self.a_desired,
      v_cruise,
      sm
    )

    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    # ✅ FIX: parse_model 시그니처/호출 일치
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error, v_ego)

    self.mpc.update(
      sm['carState'],
      sm['radarState'],
      sm['modelV2'],
      sm['controlsState'],
      v_cruise,
      x, v, a, j,
      prev_accel_constraint,
      reset_state
    )

    self.v_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill and not reset_state
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate dt seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(self.dt, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm, return_msg=False):
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.distances = self.x_desired_trajectory.tolist()
    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.visionTurnControllerState = self.vision_turn_controller.state
    longitudinalPlan.visionTurnSpeed = float(self.vision_turn_controller.v_target)
    longitudinalPlan.visionCurrentLatAcc = float(self.vision_turn_controller.current_lat_acc)
    longitudinalPlan.visionMaxPredLatAcc = float(self.vision_turn_controller.max_pred_lat_acc)
    longitudinalPlan.eventsDEPRECATED = self.events.to_msg()

    longitudinalPlan.xState = int(self.mpc.xState)
    if self.mpc.trafficError:
      longitudinalPlan.trafficState = self.mpc.trafficState + 1000
    longitudinalPlan.xStop = float(self.mpc.stopDist)
    longitudinalPlan.tFollow = float(self.mpc.t_follow)
    longitudinalPlan.cruiseGap = float(self.mpc.applyCruiseGap)
    longitudinalPlan.xObstacle = float(self.mpc.x_obstacle_min[0])
    longitudinalPlan.mpcEvent = self.mpc.mpcEvent
    longitudinalPlan.mpcMode = 1 if self.mpc.mode == 'blended' else 0

    if self.CP.openpilotLongitudinalControl:
      longitudinalPlan.xCruiseTarget = float(self.mpc.v_cruise / self.vCluRatio)
    else:
      longitudinalPlan.xCruiseTarget = float(longitudinalPlan.speeds[-1] / self.vCluRatio)

    longitudinalPlan.solverExecutionTime = float(self.mpc.solve_time)

    if return_msg:
      return plan_send

    pm.send('longitudinalPlan', plan_send)

  def cruise_solutions(self, enabled, v_ego, a_ego, v_cruise, sm):
    # Update controllers
    self.vision_turn_controller.update(enabled, v_ego, v_cruise, sm)
    self.events = Events()

    v_tsc_target = self.vision_turn_controller.v_target if self.vision_turn_controller.is_active else 255

    # Pick solution with the lowest velocity target.
    v_solutions = min(v_cruise, v_tsc_target)

    return v_solutions
