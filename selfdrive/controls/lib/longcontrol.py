#!/usr/bin/env python3
import os
import math
from typing import SupportsFloat

from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from common.conversions import Conversions as CV
from panda import ALTERNATIVE_EXPERIENCE
from selfdrive.swaglog import cloudlog
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
from selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from selfdrive.controls.lib.longcontrol import LongControl
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager, set_offroad_alert
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.hardware import HARDWARE, TICI, EON
from selfdrive.manager.process_config import managed_processes
from selfdrive.car.hyundai.scc_smoother import SccSmoother
from selfdrive.controls.ntune import ntune_common_get, ntune_common_enabled, ntune_scc_get
from decimal import Decimal

SR_SCALE_BP = [0., 40., 60., 80., 100.]
SR_SCALE_V = [16.0, 15.9, 15.6, 14.8, 13.5]

SOFT_DISABLE_TIME = 3  # seconds
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
IGNORE_PROCESSES = {"rtshield", "uploader", "deleter", "loggerd", "logmessaged", "tombstoned",
                    "logcatd", "proclogd", "clocksd", "updated", "timezoned", "manage_athenad",
                    "statsd", "shutdownd"} | \
                   {k for k, v in managed_processes.items() if not v.enabled}

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.LateralPlan.Desire
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName
ButtonEvent = car.CarState.ButtonEvent
SafetyModel = car.CarParams.SafetyModel

# ✅ (삭제) xState 기반 softHold 사용 안함
# XState = log.LongitudinalPlan.XState

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
CSID_MAP = {"1": EventName.roadCameraError, "2": EventName.wideRoadCameraError, "0": EventName.driverCameraError}
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)

LongControlState = car.CarControl.Actuators.LongControlState


class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
    config_realtime_process(4 if TICI else 3, Priority.CTRL_HIGH)

    # Setup sockets
    self.pm = pm
    if self.pm is None:
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])

    self.camera_packets = ["roadCameraState", "driverCameraState"]
    if TICI:
      self.camera_packets.append("wideRoadCameraState")

    self.can_sock = can_sock
    if can_sock is None:
      can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

    if TICI:
      self.log_sock = messaging.sub_sock('androidLog')

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      get_one_can(self.can_sock)

      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'])
    else:
      self.CI, self.CP = CI, CI.CP

    params = Params()
    self.joystick_mode = params.get_bool("JoystickDebugMode") or (self.CP.notCar and sm is None)
    joystick_packet = ['testJoystick'] if self.joystick_mode else []

    self.sm = sm
    if self.sm is None:
      ignore = ['driverCameraState', 'managerState'] if SIMULATION else None
      self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                                     'driverMonitoringState', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman',
                                     'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters'] + self.camera_packets + joystick_packet,
                                    ignore_alive=ignore, ignore_avg_freq=['radarState', 'longitudinalPlan'])

    # set alternative experiences from parameters
    self.disengage_on_accelerator = params.get_bool("DisengageOnAccelerator")
    self.CP.alternativeExperience = 0
    if not self.disengage_on_accelerator:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS

    # read params
    self.is_metric = params.get_bool("IsMetric")
    self.is_ldw_enabled = params.get_bool("IsLdwEnabled")
    openpilot_enabled_toggle = params.get_bool("OpenpilotEnabledToggle")
    self.average_desired_curvature = self.CP.pfeiferjDesiredCurvatures
    passive = params.get_bool("Passive") or not openpilot_enabled_toggle
    self.set_speed_offset = params.get_bool("SetSpeedOffset") * (1 if self.is_metric else CV.MPH_TO_KPH)

    # detect sound card presence and ensure successful init
    sounds_available = HARDWARE.get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'

    controller_available = self.CI.CC is not None and not passive and not self.CP.dashcamOnly
    self.read_only = not car_recognized or not controller_available or self.CP.dashcamOnly
    if self.read_only:
      safety_config = car.CarParams.SafetyConfig.new_message()
      safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    # Write previous route's CarParams
    prev_cp = params.get("CarParamsPersistent")
    if prev_cp is not None:
      params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for radard
    cp_bytes = self.CP.to_bytes()
    params.put("CarParams", cp_bytes)
    put_nonblocking("CarParamsCache", cp_bytes)
    put_nonblocking("CarParamsPersistent", cp_bytes)

    self.CC = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)

    self.lateral_control_select = 0
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
      self.lateral_control_select = 0
    elif self.CP.lateralTuning.which() == 'indi':
      self.LaC = LatControlINDI(self.CP, self.CI)
      self.lateral_control_select = 1
    elif self.CP.lateralTuning.which() == 'lqr':
      self.LaC = LatControlLQR(self.CP, self.CI)
      self.lateral_control_select = 2
    elif self.CP.lateralTuning.which() == 'torque' and self.CP.steerControlType != car.CarParams.SteerControlType.angle:
      self.LaC = LatControlTorque(self.CP, self.CI)
      self.lateral_control_select = 3

    self.initialized = False
    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.can_rcv_error = False
    self.soft_disable_timer = 0
    self.v_cruise_kph = 255
    self.v_cruise_kph_last = 0
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.can_rcv_error_counter = 0
    self.last_blinker_frame = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]
    self.logged_comm_issue = False
    self.button_timers = {ButtonEvent.Type.decelCruise: 0, ButtonEvent.Type.accelCruise: 0}
    self.last_actuators = car.CarControl.Actuators.new_message()
    self.steer_limited = False
    self.desired_curvature = 0.0
    self.desired_curvature_rate = 0.0
    self.nn_alert_shown = False

    # ====== longControlState 통일 변수 ======
    self.long_control_state = LongControlState.off
    self.stopping = False

    # scc smoother
    self.is_cruise_enabled = False
    self.applyMaxSpeed = 0
    self.apply_accel = 0.
    self.fused_accel = 0.
    self.lead_drel = 0.
    self.aReqValue = 0.
    self.aReqValueMin = 0.
    self.aReqValueMax = 0.
    self.sccStockCamStatus = 0
    self.sccStockCamAct = 0

    self.left_lane_visible = False
    self.right_lane_visible = False

    self.wide_camera = TICI and params.get_bool('EnableWideCamera')
    self.disable_op_fcw = params.get_bool('DisableOpFcw')
    self.mad_mode_enabled = Params().get_bool('MadModeEnabled')

    # >>> MOD: 롱크루즈갭 + T-follow 값 보관
    self.longCruiseGap = 1
    self.dRel = 0.0
    self.vRel = 0.0

    # TODO: no longer necessary, aside from process replay
    self.sm['liveParameters'].valid = True

    self.startup_event = get_startup_event(car_recognized, controller_available, len(self.CP.carFw) > 0)

    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      if len(self.CP.carFw) > 0:
        set_offroad_alert("Offroad_CarUnrecognized", True)
      else:
        set_offroad_alert("Offroad_NoFirmware", True)
    elif self.read_only:
      self.events.add(EventName.dashcamMode, static=True)
    elif self.joystick_mode:
      self.events.add(EventName.joystickDebug, static=True)
      self.startup_event = None

    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.prof = Profiler(False)

  # ---- (중간 함수들: update_events / data_sample / state_transition / state_control)
  # ---- 사용자가 올린 코드와 동일하므로 그대로 두고, publish_logs의 softHold만 바꾼 버전입니다.
  # ---- 아래는 publish_logs만 핵심 수정된 전체 반영입니다.

  def publish_logs(self, CS, start_time, CC, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    orientation_value = list(self.sm['liveLocationKalman'].calibratedOrientationNED.value)
    if len(orientation_value) > 2:
      CC.orientationNED = orientation_value
    angular_rate_value = list(self.sm['liveLocationKalman'].angularVelocityCalibrated.value)
    if len(angular_rate_value) > 2:
      CC.angularVelocity = angular_rate_value

    CC.cruiseControl.override = self.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = self.CP.pcmCruise and not self.enabled and CS.cruiseState.enabled
    if self.joystick_mode and self.sm.rcv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
      CC.cruiseControl.cancel = True

    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = self.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1

    hudControl = CC.hudControl
    hudControl.setSpeed = float(self.v_cruise_kph * CV.KPH_TO_MS)
    hudControl.speedVisible = self.enabled
    hudControl.lanesVisible = self.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead

    # ✅ (핵심) softHold는 LongControl 기반 플래그만 사용
    hudControl.softHold = bool(CC.longActive and bool(getattr(self.LoC, "softHold", False)))

    # >>> MOD: HUD에 롱갭/티팔로우 값 송출
    hudControl.cruiseGap = clip(int(self.sm['longitudinalPlan'].cruiseGap), 1, 4)
    hudControl.objDist = int(self.dRel)
    hudControl.objRelSpd = float(self.vRel)

    # ---- 이하 코드는 사용자가 올린 publish_logs와 동일 ----
    right_lane_visible = self.sm['lateralPlan'].rProb > 0.5
    left_lane_visible = self.sm['lateralPlan'].lProb > 0.5

    if self.sm.frame % 100 == 0:
      self.right_lane_visible = right_lane_visible
      self.left_lane_visible = left_lane_visible

    hudControl.rightLaneVisible = self.right_lane_visible
    hudControl.leftLaneVisible = self.left_lane_visible

    recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                  and not CC.latActive and self.sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.calibrated

    model_v2 = self.sm['modelV2']
    desire_prediction = model_v2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = self.sm['lateralPlan'].rProb > 0.5
      left_lane_visible = self.sm['lateralPlan'].lProb > 0.5
      l_lane_change_prob = desire_prediction[Desire.laneChangeLeft - 1]
      r_lane_change_prob = desire_prediction[Desire.laneChangeRight - 1]

      lane_lines = model_v2.laneLines
      l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

      hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

    if hudControl.rightLaneDepart or hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)

    clear_event_types = set()
    if ET.WARNING not in self.current_alert_types:
      clear_event_types.add(ET.WARNING)
    if self.enabled:
      clear_event_types.add(ET.NO_ENTRY)

    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, self.sm, self.is_metric, self.soft_disable_timer])
    self.AM.add_many(self.sm.frame, alerts)
    current_alert = self.AM.process_alerts(self.sm.frame, clear_event_types)
    if current_alert:
      hudControl.visualAlert = current_alert.visual_alert

    if not self.read_only and self.initialized:
      self.last_actuators, can_sends = self.CI.apply(CC, self)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
      CC.actuatorsOutput = self.last_actuators
      self.steer_limited = abs(CC.actuators.steer - CC.actuatorsOutput.steer) > 1e-2

    force_decel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                  (self.state == State.softDisabling)

    params = self.sm['liveParameters']
    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetDeg)
    curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, params.roll)

    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    controlsState = dat.controlsState
    if current_alert:
      controlsState.alertText1 = current_alert.alert_text_1
      controlsState.alertText2 = current_alert.alert_text_2
      controlsState.alertSize = current_alert.alert_size
      controlsState.alertStatus = current_alert.alert_status
      controlsState.alertBlinkingRate = current_alert.alert_rate
      controlsState.alertType = current_alert.alert_type
      controlsState.alertSound = current_alert.audible_alert

    controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    controlsState.lateralPlanMonoTime = self.sm.logMonoTime['lateralPlan']
    controlsState.enabled = self.enabled
    controlsState.active = self.active
    controlsState.curvature = curvature
    controlsState.desiredCurvature = self.desired_curvature
    controlsState.desiredCurvatureRate = self.desired_curvature_rate
    controlsState.state = self.state
    controlsState.engageable = not self.events.any(ET.NO_ENTRY)

    controlsState.longControlState = self.long_control_state

    controlsState.vPid = float(self.LoC.v_pid)
    controlsState.vCruise = float(self.applyMaxSpeed if self.CP.openpilotLongitudinalControl else self.v_cruise_kph)
    controlsState.upAccelCmd = float(self.LoC.pid.p)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    controlsState.startMonoTime = int(start_time * 1e9)
    controlsState.forceDecel = bool(force_decel)
    controlsState.canErrorCounter = self.can_rcv_error_counter
    controlsState.distanceTraveled = self.distance_traveled

    controlsState.angleSteers = steer_angle_without_offset * CV.RAD_TO_DEG
    controlsState.applyAccel = self.apply_accel
    controlsState.aReqValue = self.aReqValue
    controlsState.aReqValueMin = self.aReqValueMin
    controlsState.aReqValueMax = self.aReqValueMax
    controlsState.sccStockCamAct = self.sccStockCamAct
    controlsState.sccStockCamStatus = self.sccStockCamStatus

    controlsState.steerRatio = self.VM.sR
    controlsState.steerActuatorDelay = ntune_common_get('steerActuatorDelay')

    controlsState.sccGasFactor = ntune_scc_get('sccGasFactor')
    controlsState.sccBrakeFactor = ntune_scc_get('sccBrakeFactor')
    controlsState.sccCurvatureFactor = ntune_scc_get('sccCurvatureFactor')
    controlsState.lateralControlSelect = int(self.lateral_control_select)

    controlsState.longCruiseGap = clip(int(self.longCruiseGap), 1, 4)

    if self.joystick_mode:
      controlsState.lateralControlState.debugState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      controlsState.lateralControlState.angleState = lac_log
    elif self.CP.lateralTuning.which() == 'pid':
      controlsState.lateralControlState.pidState = lac_log
    elif self.CP.lateralTuning.which() == 'indi':
      controlsState.lateralControlState.indiState = lac_log
    elif self.CP.lateralTuning.which() == 'lqr':
      controlsState.lateralControlState.lqrState = lac_log
    elif self.CP.lateralTuning.which() == 'torque':
      controlsState.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    car_events = self.events.to_msg()
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.events = car_events
    self.pm.send('carState', cs_send)

    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('carEvents', len(self.events))
      ce_send.carEvents = car_events
      self.pm.send('carEvents', ce_send)
    self.events_prev = self.events.names.copy()

    if (self.sm.frame % int(50. / DT_CTRL) == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

    self.CC = CC

  def step(self):
    start_time = sec_since_boot()
    self.prof.checkpoint("Ratekeeper", ignore=True)

    CS = self.data_sample()
    cloudlog.timestamp("Data sampled")
    self.prof.checkpoint("Sample")

    self.update_events(CS)
    cloudlog.timestamp("Events updated")

    if not self.read_only and self.initialized:
      self.state_transition(CS)
      self.prof.checkpoint("State transition")

    CC, lac_log = self.state_control(CS)
    self.prof.checkpoint("State Control")

    self.publish_logs(CS, start_time, CC, lac_log)
    self.prof.checkpoint("Sent")

    self.update_button_timers(CS.buttonEvents)
    self.CS_prev = CS

  def controlsd_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()


def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
