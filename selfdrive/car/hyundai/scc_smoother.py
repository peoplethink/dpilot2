import copy
import random
from math import sqrt

import numpy as np
from common.numpy_fast import clip, interp, mean
from cereal import car
from common.realtime import DT_CTRL
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import Buttons
from common.params import Params
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_MIN, V_CRUISE_DELTA_KM, V_CRUISE_DELTA_MI, \
  CONTROL_N
from selfdrive.controls.ntune import ntune_scc_get
from selfdrive.controls.lib.lane_planner import TRAJECTORY_SIZE
from common.filter_simple import StreamingMovingAverage
from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed, road_speed_limiter_get_active, \
  get_road_speed_limiter

SYNC_MARGIN = 5.

# do not modify
MIN_SET_SPEED_KPH = V_CRUISE_MIN
MAX_SET_SPEED_KPH = V_CRUISE_MAX

ALIVE_COUNT = [8, 10]
WAIT_COUNT = [12, 14, 16, 18]
AliveIndex = 0
WaitIndex = 0

MIN_CURVE_SPEED = 20. * CV.KPH_TO_MS

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

# 국가법령정보센터: 도로설계기준
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]

class SccSmoother:

  @staticmethod
  def get_alive_count():
    global AliveIndex
    count = ALIVE_COUNT[AliveIndex]
    AliveIndex += 1
    if AliveIndex >= len(ALIVE_COUNT):
      AliveIndex = 0
    return count

  @staticmethod
  def get_wait_count():
    global WaitIndex
    count = WAIT_COUNT[WaitIndex]
    WaitIndex += 1
    if WaitIndex >= len(WAIT_COUNT):
      WaitIndex = 0
    return count

  def kph_to_clu(self, kph):
    return int(kph * CV.KPH_TO_MS * self.speed_conv_to_clu)

  def __init__(self):
    # Params 객체 재사용 (실시간 갱신용)
    self.params = Params()
    self._params_frame = -1

    self.longcontrol = self.params.get_bool('LongControlEnabled')
    self.slow_on_curves = self.params.get_bool('SccSmootherSlowOnCurves')
    self.autoCurveSpeedFactor = float(int(self.params.get("AutoCurveSpeedFactor", encoding="utf8"))) * 0.01
    self.autoCurveSpeedFactorIn = float(int(self.params.get("AutoCurveSpeedFactorIn", encoding="utf8"))) * 0.01
    self.sync_set_speed_while_gas_pressed = self.params.get_bool('SccSmootherSyncGasPressed')
    self.is_metric = self.params.get_bool('IsMetric')
    self.autoascc = self.params.get_bool('AutoAscc')

    self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.min_set_speed_clu = self.kph_to_clu(MIN_SET_SPEED_KPH)
    self.max_set_speed_clu = self.kph_to_clu(MAX_SET_SPEED_KPH)

    self.target_speed = 0.

    self.started_frame = 0
    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE

    self.alive_count = ALIVE_COUNT
    random.shuffle(WAIT_COUNT)

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False
    self.active_cam = False
    self.over_speed_limit = False

    self.max_speed_clu = 0.
    self.limited_lead = False

    self.curve_speed_ms = 0.
    self.stock_weight = 0.

    self.turnSpeed_prev = 300
    self.curvatureFilter = StreamingMovingAverage(20)

  def update_params_3(self, frame: int):
    if frame == self._params_frame:
      return
    if frame % 20 != 0:
      return
    self._params_frame = frame

    self.slow_on_curves = self.params.get_bool('SccSmootherSlowOnCurves')
    self.autoCurveSpeedFactor = float(int(self.params.get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
    self.autoCurveSpeedFactorIn = float(int(self.params.get("AutoCurveSpeedFactorIn", encoding="utf8")))*0.01

  def reset(self):

    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE
    self.target_speed = 0.

    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False

  @staticmethod
  def create_clu11(packer, bus, clu11, button):
    values = copy.copy(clu11)
    values["CF_Clu_CruiseSwState"] = button
    values["CF_Clu_AliveCnt1"] = (values["CF_Clu_AliveCnt1"] + 1) % 0x10
    return packer.make_can_msg("CLU11", bus, values)

  def is_active(self, frame):
    return frame - self.started_frame <= max(ALIVE_COUNT) + max(WAIT_COUNT)

  def inject_events(self, events):
    if self.slowing_down_sound_alert:
      self.slowing_down_sound_alert = False
      events.add(EventName.slowingDownSpeedSound)
    elif self.slowing_down_alert:
      events.add(EventName.slowingDownSpeed)

  def cal_max_speed(self, frame, CC, CS, sm, clu11_speed, controls):
    road_speed_limiter = get_road_speed_limiter()
    apply_limit_speed, road_limit_speed, left_dist, first_started, cam_type, max_speed_log = \
      road_speed_limiter.get_max_speed(clu11_speed, self.is_metric)

    curv_limit = 0
    self.cal_curve_speed(sm, CS.out.vEgo, frame)
    if self.slow_on_curves and self.curve_speed_ms >= MIN_CURVE_SPEED:
      max_speed_clu = min(controls.v_cruise_kph * CV.KPH_TO_MS, self.curve_speed_ms) * self.speed_conv_to_clu
      curv_limit = int(max_speed_clu)
    else:
      max_speed_clu = self.kph_to_clu(controls.v_cruise_kph)

    self.active_cam = road_limit_speed > 0 and left_dist > 0

    if road_speed_limiter.roadLimitSpeed is not None:
      camSpeedFactor = clip(road_speed_limiter.roadLimitSpeed.camSpeedFactor, 1.0, 1.1)
      self.over_speed_limit = road_speed_limiter.roadLimitSpeed.camLimitSpeedLeftDist > 0 and \
                              0 < road_limit_speed * camSpeedFactor < clu11_speed + 2
    else:
      self.over_speed_limit = False

    max_speed_log = ""

    if apply_limit_speed >= self.kph_to_clu(10):

      if first_started:
        self.max_speed_clu = clu11_speed

      max_speed_clu = min(max_speed_clu, apply_limit_speed)

      if clu11_speed > apply_limit_speed:

        if not self.slowing_down_alert and not self.slowing_down:
          self.slowing_down_sound_alert = True
          self.slowing_down = True

        self.slowing_down_alert = True

      else:
        self.slowing_down_alert = False

    else:
      self.slowing_down_alert = False
      self.slowing_down = False

    lead_speed = self.get_long_lead_speed(CS, clu11_speed, sm)

    if lead_speed >= self.min_set_speed_clu:
      if lead_speed < max_speed_clu:
        max_speed_clu = min(max_speed_clu, lead_speed)

        if not self.limited_lead:
          self.max_speed_clu = clu11_speed + 3.
          self.limited_lead = True
    else:
      self.limited_lead = False

    self.update_max_speed(int(max_speed_clu + 0.5),
                          curv_limit != 0 and curv_limit == int(max_speed_clu))

    return road_limit_speed, left_dist, max_speed_log

  def update(self, enabled, can_sends, packer, CC, CS, frame, controls):
    # ✅ 3개값 실시간 갱신 (slow_on_curves / factor / factorIn)
    self.update_params_3(frame)

    # mph or kph
    clu11_speed = CS.clu11["CF_Clu_Vanz"]

    road_limit_speed, left_dist, max_speed_log = self.cal_max_speed(frame, CC, CS, controls.sm, clu11_speed, controls)

    # kph
    controls.applyMaxSpeed = float(clip(CS.cruiseState_speed * CV.MS_TO_KPH, MIN_SET_SPEED_KPH,
                                                self.max_speed_clu * self.speed_conv_to_ms * CV.MS_TO_KPH))
    CC.sccSmoother.longControl = self.longcontrol
    CC.sccSmoother.applyMaxSpeed = controls.applyMaxSpeed
    CC.sccSmoother.cruiseMaxSpeed = controls.v_cruise_kph

    ascc_enabled = CS.acc_mode and enabled and CS.cruiseState_enabled \
                   and 1 < CS.cruiseState_speed < 255 and not CS.brake_pressed

    # Auto-resume Cruise Set Speed by JangPoo
    dRel = 0.
    lead = self.get_lead(controls.sm)
    if lead is not None:
      dRel = lead.dRel

    # Auto-resume Cruise Set Speed by JangPoo
    ascc_auto_set = enabled and (clu11_speed > 30 or (CS.obj_valid and dRel > 1)) \
                    and CS.gas_pressed and CS.prev_cruiseState_speed and not CS.cruiseState_speed

    if not self.longcontrol:
      if (not ascc_enabled or CS.standstill or CS.cruise_buttons != Buttons.NONE) and not ascc_auto_set:
        self.reset()
        self.wait_timer = max(ALIVE_COUNT) + max(WAIT_COUNT)
        return

    if not ascc_enabled and not ascc_auto_set:
      self.reset()

    self.cal_target_speed(CS, clu11_speed, controls)

    CC.sccSmoother.logMessage = max_speed_log

    if self.wait_timer > 0:
      self.wait_timer -= 1
    elif (ascc_enabled and not CS.out.cruiseState.standstill) or ascc_auto_set:
      if self.alive_timer == 0:
        if ascc_enabled:
          if self.autoascc:
            self.btn = self.get_button(CS.cruiseState_speed * self.speed_conv_to_clu)
        elif ascc_auto_set and clu11_speed < 30:
          if self.autoascc:
            self.btn = Buttons.SET_DECEL
        else:
          self.btn = Buttons.RES_ACCEL
        self.alive_count = SccSmoother.get_alive_count()

      if self.btn != Buttons.NONE:

        can_sends.append(SccSmoother.create_clu11(packer, CS.scc_bus, CS.clu11, self.btn))

        if self.alive_timer == 0:
          self.started_frame = frame

        self.alive_timer += 1

        if self.alive_timer >= self.alive_count:
          self.alive_timer = 0
          self.wait_timer = SccSmoother.get_wait_count()
          self.btn = Buttons.NONE
      else:
        if self.longcontrol and self.target_speed >= self.min_set_speed_clu:
          self.target_speed = 0.
    else:
      if self.longcontrol:
        self.target_speed = 0.

  def get_button(self, current_set_speed):

    if self.target_speed < self.min_set_speed_clu:
      return Buttons.NONE

    error = self.target_speed - current_set_speed
    if abs(error) < 0.9:
      return Buttons.NONE

    return Buttons.RES_ACCEL if error > 0 else Buttons.SET_DECEL

  def get_lead(self, sm):

    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne

    return None

  def get_long_lead_speed(self, CS, clu11_speed, sm):

    if self.longcontrol:
      lead = self.get_lead(sm)
      if lead is not None:
        d = lead.dRel - 5.
        if 0. < d < -lead.vRel * (9. + 3.) * 2. and lead.vRel < -1.:
          t = d / lead.vRel
          accel = -(lead.vRel / t) * self.speed_conv_to_clu
          accel *= 0.8

          if accel < 0.:
            target_speed = clu11_speed + accel
            target_speed = max(target_speed, self.min_set_speed_clu)
            return target_speed

    return 0

  def cal_curve_speed(self, sm, v_ego, frame):
    # 회전속도를 선속도 나누면 : 곡률이 됨. [20]은 약 4초앞의 곡률을 보고 커브를 계산함.
    #curvature = abs(controls.sm['modelV2'].orientationRate.z[20] / clip(CS.vEgo, 0.1, 100.0))
    orientationRates = np.array(controls.sm['modelV2'].orientationRate.z, dtype=np.float32)
    # 계산된 결과로, oritetationRates를 나누어 조금더 curvature값이 커지도록 함.
    speed = min(self.turnSpeed_prev / 3.6, clip(CS.vEgo, 0.5, 100.0))    
    #curvature = np.max(np.abs(orientationRates[12:])) / speed  # 12: 약1.4초 미래의 curvature를 계산함.
    curvature = np.max(np.abs(orientationRates[12:20])) / speed  # 12: 약1.4~3.5초 미래의 curvature를 계산함.
    curvature = self.curvatureFilter.process(curvature) * self.autoCurveSpeedFactor
    turnSpeed = 300
    if abs(curvature) > 0.0001:
      turnSpeed = interp(curvature, V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
      turnSpeed = clip(turnSpeed, MIN_CURVE_SPEED, 255)
    else:
      turnSpeed = 300

    self.turnSpeed_prev = turnSpeed
    speed_diff = max(0, CS.vEgo*3.6 - turnSpeed)
    turnSpeed = turnSpeed - speed_diff * self.autoCurveSpeedFactorIn
    controls.debugText2 = 'CURVE={:5.1f},curvature={:5.4f},mode={:3.1f}'.format(self.turnSpeed_prev, curvature, self.drivingModeIndex)
    return turnSpeed

  def cal_target_speed(self, CS, clu11_speed, controls):

    if not self.longcontrol:
      if CS.gas_pressed and self.sync_set_speed_while_gas_pressed and CS.cruise_buttons == Buttons.NONE:
        if clu11_speed + SYNC_MARGIN > self.kph_to_clu(controls.v_cruise_kph):
          set_speed = clip(clu11_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          controls.v_cruise_kph = set_speed * self.speed_conv_to_ms * CV.MS_TO_KPH

      self.target_speed = self.kph_to_clu(controls.v_cruise_kph)

      if self.max_speed_clu > self.min_set_speed_clu:
        self.target_speed = clip(self.target_speed, self.min_set_speed_clu, self.max_speed_clu)

    elif CS.cruiseState_enabled:
      if CS.gas_pressed and self.sync_set_speed_while_gas_pressed and CS.cruise_buttons == Buttons.NONE:
        if clu11_speed + SYNC_MARGIN > self.kph_to_clu(controls.v_cruise_kph):
          set_speed = clip(clu11_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          self.target_speed = set_speed

  def update_max_speed(self, max_speed, limited_curv):

    if not self.longcontrol or self.max_speed_clu <= 0:
      self.max_speed_clu = max_speed
    else:
      kp = 0.01
      error = max_speed - self.max_speed_clu
      self.max_speed_clu = self.max_speed_clu + error * kp

  def get_apply_accel(self, CS, sm, accel, stopping):

    gas_factor = ntune_scc_get("sccGasFactor")
    brake_factor = ntune_scc_get("sccBrakeFactor")

    if accel > 0:
      accel *= gas_factor
    else:
      accel *= brake_factor

    return accel

  def get_stock_cam_accel(self, apply_accel, stock_accel, scc11):
    stock_cam = scc11["Navi_SCC_Camera_Act"] == 2 and scc11["Navi_SCC_Camera_Status"] == 2
    if stock_cam:
      self.stock_weight += DT_CTRL / 3.
    else:
      self.stock_weight -= DT_CTRL / 3.

    self.stock_weight = clip(self.stock_weight, 0., 1.)

    accel = stock_accel * self.stock_weight + apply_accel * (1. - self.stock_weight)
    return min(accel, apply_accel), stock_cam

  @staticmethod
  def update_cruise_buttons(controls, CS, longcontrol):  # called by controlds's state_transition

    car_set_speed = CS.cruiseState.speed * CV.MS_TO_KPH
    is_cruise_enabled = car_set_speed != 0 and car_set_speed != 255 and CS.cruiseState.enabled and controls.CP.pcmCruise

    if is_cruise_enabled:
      if longcontrol:
        v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      else:
        v_cruise_kph = SccSmoother.update_v_cruise(controls.v_cruise_kph, CS.buttonEvents, controls.enabled,
                                                   controls.is_metric)
    else:
      v_cruise_kph = 0

    if controls.is_cruise_enabled != is_cruise_enabled:
      controls.is_cruise_enabled = is_cruise_enabled

      if controls.is_cruise_enabled:
        v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      else:
        v_cruise_kph = 0

      controls.LoC.reset(v_pid=CS.vEgo)

    controls.v_cruise_kph = v_cruise_kph

  @staticmethod
  def update_v_cruise(v_cruise_kph, buttonEvents, enabled, metric):

    global ButtonCnt, LongPressed, ButtonPrev
    if enabled:
      if ButtonCnt:
        ButtonCnt += 1
      for b in buttonEvents:
        if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or b.type == ButtonType.decelCruise):
          ButtonCnt = 1
          ButtonPrev = b.type
        elif not b.pressed and ButtonCnt:
          if not LongPressed and b.type == ButtonType.accelCruise:
            v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
          elif not LongPressed and b.type == ButtonType.decelCruise:
            v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
          LongPressed = False
          ButtonCnt = 0
      if ButtonCnt > 70:
        LongPressed = True
        V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
        if ButtonPrev == ButtonType.accelCruise:
          v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
        elif ButtonPrev == ButtonType.decelCruise:
          v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
        ButtonCnt %= 70
      v_cruise_kph = clip(v_cruise_kph, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)

    return v_cruise_kph
