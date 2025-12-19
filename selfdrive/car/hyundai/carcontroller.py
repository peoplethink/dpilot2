from random import randint

from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits, common_fault_avoidance
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, \
  create_scc11, create_scc12, create_scc13, create_scc14, \
  create_mdps12, create_lfahda_mfc, create_hda_mfc
from selfdrive.car.hyundai.scc_smoother import SccSmoother
from selfdrive.car.hyundai.values import Buttons, CAR, FEATURES, CarControllerParams, LEGACY_SAFETY_MODE_CAR
from opendbc.can.packer import CANPacker
from common.conversions import Conversions as CV
from common.params import Params
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.road_speed_limiter import road_speed_limiter_get_active

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
min_set_speed = 30 * CV.KPH_TO_MS

MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1
  if hud_control.rightLaneDepart:
    right_lane_warning = 1

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.apply_steer_last = 0
    self.accel = 0
    self.accel_last = 0

    self.lkas11_cnt = 0
    self.scc12_cnt = -1

    self.resume_cnt = 0
    self.last_lead_distance = 0
    self.resume_wait_timer = 0

    self.last_button_frame = 0
    self.button_wait = 12
    self.button_alive = 0
    self.button_alive_frame = 0

    self.turning_signal_timer = 0
    self.longcontrol = CP.openpilotLongitudinalControl
    self.scc_live = not CP.radarOffCan

    self.turning_indicator_alert = False

    param = Params()
    self.mad_mode_enabled = param.get_bool('MadModeEnabled')
    self.ldws_opt = param.get_bool('IsLdwsCar')
    self.stock_navi_decel_enabled = param.get_bool('StockNaviDecelEnabled')
    self.keep_steering_turn_signals = param.get_bool('KeepSteeringTurnSignals')
    self.haptic_feedback_speed_camera = param.get_bool('HapticFeedbackWhenSpeedCamera')

    self.scc_smoother = SccSmoother()
    self.prev_active_cam = False
    self.active_cam_timer = 0
    self.last_active_cam_frame = 0

    self.maxAngleFrames = MAX_ANGLE_FRAMES
    self.angle_limit_counter = 0

    self.jerkStartLimit = 1.0
    self.jerk_count = 0.0

  def update(self, CC, CS, controls):
    actuators = CC.actuators
    hud_control = CC.hudControl

    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(
      abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
      self.angle_limit_counter, self.maxAngleFrames,
      MAX_ANGLE_CONSECUTIVE_FRAMES
    )

    lkas_active = CC.latActive

    if CS.out.leftBlinker or CS.out.rightBlinker:
      self.turning_signal_timer = 0.5 / DT_CTRL
    if self.turning_indicator_alert:
      lkas_active = 0
    if self.turning_signal_timer > 0:
      self.turning_signal_timer -= 1

    if not lkas_active:
      apply_steer = 0

    torque_fault = CC.latActive and not apply_steer_req
    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(
      CC.enabled, self.car_fingerprint, hud_control
    )

    if self.haptic_feedback_speed_camera:
      if self.prev_active_cam != self.scc_smoother.active_cam:
        self.prev_active_cam = self.scc_smoother.active_cam
        if self.scc_smoother.active_cam:
          if (self.frame - self.last_active_cam_frame) * DT_CTRL > 10.0:
            self.active_cam_timer = int(1.5 / DT_CTRL)
            self.last_active_cam_frame = self.frame

      if self.active_cam_timer > 0:
        self.active_cam_timer -= 1
        left_lane_warning = right_lane_warning = 1

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph else 60
    if clu11_speed > enabled_speed or not lkas_active:
      enabled_speed = clu11_speed

    if self.frame == 0:
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"]
    self.lkas11_cnt = (self.lkas11_cnt + 1) % 0x10

    # ===== Params 주기 로드 =====
    if self.frame % 100 == 0:
      self.maxAngleFrames = int(Params().get("MaxAngleFrames", encoding="utf8"))
      self.jerkStartLimit = float(int(Params().get("JerkStartLimit", encoding="utf8"))) * 0.1

    can_sends = []

    can_sends.append(create_lkas11(
      self.packer, self.frame, self.car_fingerprint, apply_steer, apply_steer_req,
      torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
      hud_control.leftLaneVisible, hud_control.rightLaneVisible,
      left_lane_warning, right_lane_warning, 0, self.ldws_opt
    ))

    if CS.mdps_bus or CS.scc_bus == 1:
      can_sends.append(create_lkas11(
        self.packer, self.frame, self.car_fingerprint, apply_steer, apply_steer_req,
        torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
        hud_control.leftLaneVisible, hud_control.rightLaneVisible,
        left_lane_warning, right_lane_warning, 1, self.ldws_opt
      ))

    if self.frame % 2 and CS.mdps_bus:
      can_sends.append(create_clu11(self.packer, CS.mdps_bus, CS.clu11, Buttons.NONE, enabled_speed))

    if CS.mdps_bus or self.car_fingerprint in FEATURES["send_mdps12"]:
      can_sends.append(create_mdps12(self.packer, self.frame, CS.mdps12))

    self.update_auto_resume(CC, CS, clu11_speed, can_sends)
    self.update_scc(CC, CS, actuators, controls, hud_control, can_sends)

    if self.frame % 5 == 0:
      activated_hda = road_speed_limiter_get_active()
      if self.car_fingerprint in FEATURES["send_lfa_mfa"]:
        can_sends.append(create_lfahda_mfc(self.packer, CC.enabled, activated_hda))
      elif CS.has_lfa_hda:
        can_sends.append(create_hda_mfc(self.packer, activated_hda, CS,
                                        hud_control.leftLaneVisible, hud_control.rightLaneVisible))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends

  def update_auto_resume(self, CC, CS, clu11_speed, can_sends):
    if CC.cruiseControl.resume and not CS.out.gasPressed:
      if self.car_fingerprint in LEGACY_SAFETY_MODE_CAR:
        if self.resume_wait_timer > 0:
          self.resume_wait_timer -= 1
        else:
          can_sends.append(create_clu11(self.packer, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
          self.resume_cnt += 1
          if self.resume_cnt >= int(randint(4, 5) * 2):
            self.resume_cnt = 0
            self.resume_wait_timer = int(randint(20, 25) * 2)
      else:
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          can_sends.append(create_clu11(self.packer, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
          self.last_button_frame = self.frame
    else:
      self.resume_wait_timer = 0
      self.resume_cnt = 0

  def update_scc(self, CC, CS, actuators, controls, hud_control, can_sends):
    self.scc_smoother.update(CC.enabled, can_sends, self.packer, CC, CS, self.frame, controls)

    if self.longcontrol and CS.cruiseState_enabled and (CS.scc_bus or not self.scc_live):
      if self.frame % 2 == 0:
        set_speed = hud_control.setSpeed
        if not (min_set_speed < set_speed < 255 * CV.KPH_TO_MS):
          set_speed = min_set_speed
        set_speed *= CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH

        # ===== accel/stopping/jerk/cb : softHold 제거(유지) + 급가속 방지 보정 =====
        apply_accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
        stopping = (actuators.longControlState == LongCtrlState.stopping)

        aReqValue = CS.scc12["aReqValue"]
        controls.aReqValue = aReqValue
        if aReqValue < controls.aReqValueMin:
          controls.aReqValueMin = controls.aReqValue
        if aReqValue > controls.aReqValueMax:
          controls.aReqValueMax = controls.aReqValue

        # (중요) stock_cam 보정은 apply_accel 확정 전에 반영하고, 확정 후 self.accel/controls.apply_accel에 넣기
        if self.stock_navi_decel_enabled:
          controls.sccStockCamAct = CS.scc11["Navi_SCC_Camera_Act"]
          controls.sccStockCamStatus = CS.scc11["Navi_SCC_Camera_Status"]
          apply_accel, stock_cam = self.scc_smoother.get_stock_cam_accel(apply_accel, aReqValue, CS.scc11)
        else:
          controls.sccStockCamAct = 0
          controls.sccStockCamStatus = 0
          stock_cam = False

        # 확정된 accel을 내부 상태/외부에 반영 (급가속 체감 튐 방지)
        self.accel = apply_accel
        self.accel_last = apply_accel
        controls.apply_accel = apply_accel

        # (핵심) softHold 없으니 "저속/정지 근처"에서 jerk_count 리셋을 stopping만 믿지 말고 보강
        low_speed = CS.out.vEgo < 1.0
        near_stop = stopping or low_speed or CS.out.brakePressed
        if near_stop:
          self.jerk_count = 0.0

        if self.scc12_cnt < 0:
          self.scc12_cnt = CS.scc12["CR_VSM_Alive"] if not CS.no_radar else 0
        self.scc12_cnt = (self.scc12_cnt + 1) % 0xF

        can_sends.append(create_scc12(
          self.packer, apply_accel, CC.enabled, self.scc12_cnt, self.scc_live, CS.scc12,
          CC.cruiseControl.override, CS.out.brakePressed, CC.cruiseControl.resume,
          self.car_fingerprint
        ))

        can_sends.append(create_scc11(
          self.packer, self.frame, CC.enabled, set_speed, hud_control.leadVisible,
          self.scc_live, CS.scc11, self.scc_smoother.active_cam, stock_cam
        ))

        if self.frame % 20 == 0 and CS.has_scc13:
          can_sends.append(create_scc13(self.packer, CS.scc13))

        if CS.has_scc14:
          # standstill 판단도 약간 보강(softHold 없이 정지/출발 안정)
          acc_standstill = (CS.out.vEgo < 0.3) or stopping

          jerk = getattr(actuators, "jerk", 0.0)
          startingJerk = self.jerkStartLimit
          jerkLimit = 5.0

          self.jerk_count += DT_CTRL
          jerk_max = interp(self.jerk_count, [0, 1.5, 2.5], [startingJerk, startingJerk, jerkLimit])

          cb_upper = 0.0
          cb_lower = 0.0

          long_state = actuators.longControlState
          if long_state == LongCtrlState.off:
            upper_jerk = jerkLimit
            lower_jerk = jerkLimit
            self.jerk_count = 0.0
          elif long_state == LongCtrlState.stopping or near_stop:
            # softHold은 제거했지만 "정지 근처"는 동일하게 출발 jerk 억제
            upper_jerk = 0.5
            lower_jerk = jerkLimit
            self.jerk_count = 0.0
          else:
            upper_jerk = min(max(0.5, jerk * 2.0), jerk_max)
            lower_jerk = min(max(1.0, -jerk * 2.0), jerk_max)
            cb_upper = clip(0.9 + apply_accel * 0.2, 0.0, 1.2)
            cb_lower = clip(0.8 + apply_accel * 0.2, 0.0, 1.2)

          lead = self.scc_smoother.get_lead(controls.sm)
          if lead is not None:
            d = float(lead.dRel)
            vrel = float(getattr(lead, "vRel", 0.0))
            obj_gap = 2 if d < 25 else 3 if d < 40 else 4 if d < 70 else 5
            obj_gap2 = 2 if vrel < -0.2 else 1
          else:
            obj_gap = 0
            obj_gap2 = 0

          can_sends.append(create_scc14(
            self.packer,
            CC.enabled,
            CS.out.vEgo,
            acc_standstill,
            apply_accel,
            upper_jerk,
            lower_jerk,
            cb_upper,
            cb_lower,
            CC.cruiseControl.override,
            obj_gap,
            obj_gap2,
            CS.scc14
          ))

    else:
      self.scc12_cnt = -1
