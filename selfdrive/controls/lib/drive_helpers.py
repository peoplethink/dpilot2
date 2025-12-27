import math

from cereal import car, log
from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.ntune import ntune_common_get

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# kph
V_CRUISE_MAX = 145
V_CRUISE_MIN = 8
V_CRUISE_DELTA_MI = 5
V_CRUISE_DELTA_KM = 10
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = 1.6

# ✅ missing constant fix (prevent NameError)
# openpilot upstream uses a minimum engage speed constant; if you don't have it, use V_CRUISE_MIN.
V_CRUISE_ENABLE_MIN = V_CRUISE_MIN

MIN_DIST = 0.001
MIN_SPEED = 1.0
LAT_MPC_N = 16
LON_MPC_N = 32
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

MAX_VEL_ERR = 5.0

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class MPC_COST_LAT:
  PATH = 1.0
  HEADING = 1.0
  STEER_RATE = 1.0


# =============================================================================
# commaai#26472: VCruiseHelper (controlsd가 set speed 관리 로직을 분리해서 사용)
# =============================================================================
class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_INITIAL
    self.v_cruise_cluster_kph = V_CRUISE_INITIAL
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False} for btn in self.button_timers}

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_INITIAL

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # stock cruise가 완전히 꺼져있는 차종이면 openpilot이 set speed를 관리
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    else:
      self.v_cruise_kph = V_CRUISE_INITIAL
      self.v_cruise_cluster_kph = V_CRUISE_INITIAL

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # NOTE: enable 되기 전에는 속도 변경하지 않음
    if not enabled:
      return

    long_press = False
    button_type = None

    # should be CV.MPH_TO_KPH, but this causes rounding errors
    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    # release edge 우선 처리
    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      # long press 유지 처리
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return
      
    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return
      
    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]  
  

    # If set/decel is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1
    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill}

  def initialize_v_cruise(self, CS):
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(clip(CS.vEgo * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


# -----------------------------------------------------------------------------
# Legacy functions (기존 코드 호환용)
# - set_speed_offset, V_CRUISE_ENABLE_MIN 누락으로 터지던 부분을 안전하게 수정
# -----------------------------------------------------------------------------
def update_v_cruise(v_cruise_kph, buttonEvents, button_timers, enabled, metric, set_speed_offset: float = 0.0):
  # handle button presses. TODO: this should be in state_control, but a decelCruise press
  # would have the effect of both enabling and changing speed is checked after the state transition
  if not enabled:
    return v_cruise_kph

  long_press = False
  button_type = None

  # should be CV.MPH_TO_KPH, but this causes rounding errors
  v_cruise_delta = 1. if metric else IMPERIAL_INCREMENT

  for b in buttonEvents:
    if b.type.raw in button_timers and not b.pressed:
      if button_timers[b.type.raw] > CRUISE_LONG_PRESS:
        return v_cruise_kph  # end long press
      button_type = b.type.raw
      break
  else:
    for k in button_timers.keys():
      if button_timers[k] and button_timers[k] % CRUISE_LONG_PRESS == 0:
        button_type = k
        long_press = True
        break

  if button_type:
    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and v_cruise_kph % v_cruise_delta != 0:  # partial interval
      v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]
    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

    # ✅ NameError 방지: set_speed_offset은 기본 0.0
    if set_speed_offset:
      v_cruise_offset = (set_speed_offset * CRUISE_INTERVAL_SIGN[button_type]) if long_press else 0.0
      if v_cruise_offset < 0:
        v_cruise_offset = set_speed_offset - v_cruise_delta
      v_cruise_kph += v_cruise_offset

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) and v_cruise_last < 250:
      return v_cruise_last

  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates, distances, average_desired_curvature):
  if len(psis) != CONTROL_N or len(distances) != CONTROL_N:
    psis = [0.0] * CONTROL_N
    curvatures = [0.0] * CONTROL_N
    curvature_rates = [0.0] * CONTROL_N
    distances = [0.0] * CONTROL_N

  v_ego = max(v_ego, 0.1)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = ntune_common_get('steerActuatorDelay') + .2

  current_curvature_desired = curvatures[0]
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)

  # Pfeiferj's #28118 PR - https://github.com/commaai/openpilot/pull/28118
  distance = interp(delay, T_IDXS[:CONTROL_N], distances)
  distance = max(MIN_DIST, distance)

  average_curvature_desired = psi / distance if average_desired_curvature else psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  desired_curvature_rate = curvature_rates[0]
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)

  safe_desired_curvature_rate = clip(desired_curvature_rate, -max_curvature_rate, max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature * 1.05,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float,
                 torque_params: car.CarParams.LateralTorqueTuning, friction_compensation: bool) -> float:
  friction_interp = interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction, torque_params.friction]
  )
  friction = float(friction_interp) if friction_compensation else 0.0
  return friction


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0
