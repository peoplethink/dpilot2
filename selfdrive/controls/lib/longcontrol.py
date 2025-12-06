from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from selfdrive.controls.lib.pid import PIDController
from selfdrive.modeld.constants import T_IDXS
from common.conversions import Conversions as CV

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill):
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)

  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off
  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf,
                             k_d=(CP.longitudinalTuning.kdBP, CP.longitudinalTuning.kdV),
                             derivative_period=0.5, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""

    # Interp control trajectory
    speeds = long_plan.speeds
    a_target_now = 0.0
    a_target = 0.0
    v_target = 0.0
    v_target_now = 0.0
    v_target_1sec = 0.0

    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      # 액추에이터 딜레이는 CP에서 세팅한 값 사용 (기본값 기준)
      longitudinalActuatorDelay = max(0.1, float(self.CP.longitudinalActuatorDelay))

      v_target = interp(longitudinalActuatorDelay + t_since_plan,
                        T_IDXS[:CONTROL_N], speeds)
      # 딜레이를 고려한 목표 가속도 재계산
      a_target = 2.0 * (v_target - v_target_now) / longitudinalActuatorDelay - a_target_now

      v_target_1sec = interp(longitudinalActuatorDelay + t_since_plan + 1.0,
                             T_IDXS[:CONTROL_N], speeds)

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel

    # 상태 머신 업데이트
    self.long_control_state = long_control_state_trans(
      self.CP, active, self.long_control_state, CS.vEgo,
      v_target, v_target_1sec, CS.brakePressed,
      CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.0

    elif self.long_control_state == LongCtrlState.stopping:
      # 정지 단계: 너무 세게 브레이크 잡지 않도록 CP.stopAccel과 CP.stoppingDecelRate 사용
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      # 출발 단계: CP.startAccel 사용
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # 정지 직전 오버슈트 방지 로직 (기존 openpilot 패턴 유지)
      prevent_overshoot = (not self.CP.stoppingControl and
                           CS.vEgo < 1.5 and
                           v_target_1sec < 0.7 and
                           v_target_1sec < self.v_pid)

      deadzone = interp(CS.vEgo,
                        self.CP.longitudinalTuning.deadzoneBP,
                        self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)

      # 타겟 가속 feedforward가 너무 커지면 브레이크를 툭 치는 현상 생길 수 있어서
      # 승용차 기준 편한 범위로 클립 (-2.0 ~ 2.0 m/s^2)
      a_ff = clip(a_target, -2.0, 2.0)

      output_accel = self.pid.update(error_deadzone,
                                     speed=CS.vEgo,
                                     feedforward=a_ff,
                                     freeze_integrator=freeze_integrator)

    # 최종 출력 클립
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
