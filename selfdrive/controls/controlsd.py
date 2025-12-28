#!/usr/bin/env python3
import os
import math
from typing import SupportsFloat
from decimal import Decimal

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

# ✅ commaai#26472: VCruiseHelper로 통일
from selfdrive.controls.lib.drive_helpers import V_CRUISE_INITIAL, VCruiseHelper, get_lag_adjusted_curvature

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
ButtonType = car.CarState.ButtonEvent.Type
SafetyModel = car.CarParams.SafetyModel

# ✅ xState 사용
XState = log.LongitudinalPlan.XState

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
CSID_MAP = {"1": EventName.roadCameraError, "2": EventName.wideRoadCameraError, "0": EventName.driverCameraError}
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)

LongControlState = car.CarControl.Actuators.LongControlState


class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
    config_realtime_process(4 if TICI else 3, Priority.CTRL_HIGH)

    # ✅ Params 통일 (크래시 방지 핵심)
    self.params = Params()
    params = self.params

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
      print("Waiting for CAN messages...")
      get_one_can(self.can_sock)
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'])
    else:
      self.CI, self.CP = CI, CI.CP

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

    if not self.CP.experimentalLongitudinalAvailable:
      self.params.delete("ExperimentalLongitudinalEnabled")
    if not self.CP.openpilotLongitudinalControl:
      self.params.delete("ExperimentalMode")

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
    else:
      # fallback (안전)
      self.LaC = LatControlPID(self.CP, self.CI)
      self.lateral_control_select = 0

    self.initialized = False
    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.can_rcv_error = False
    self.soft_disable_timer = 0
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.can_rcv_error_counter = 0
    self.last_blinker_frame = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]
    self.logged_comm_issue = False
    self.last_actuators = car.CarControl.Actuators.new_message()
    self.steer_limited = False
    self.desired_curvature = 0.0
    self.desired_curvature_rate = 0.0
    self.nn_alert_shown = False
    self.experimental_mode = False

    # ✅ commaai#26472: v_cruise helper 도입
    self.v_cruise_helper = VCruiseHelper(self.CP)

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
    self.mad_mode_enabled = params.get_bool('MadModeEnabled')

    # ✅✅ (이식) mpcEvent 디바운스 상태
    self._mpc_event_prev = 0
    self._mpc_event_frame = 0
    self._mpc_event_wait_frames = 0  # (호환용, 실제 계산은 함수에서 수행)

    # 롱크루즈갭 + lead 정보 보관
    self.longCruiseGap = 1
    self.dRel = 0.0
    self.vRel = 0.0

    # ✅✅ (이식) planner가 참조하는 myDrivingMode / mySafeModeFactor 기본값
    self.myDrivingMode = 0
    self.mySafeModeFactor = 1.0

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

  # ✅✅ (이식) CruiseHelper send_apilot_event 방식: 시간 디바운스
  def _send_mpc_event(self, mpc_evt: int, waiting_s: float = 5.0) -> None:
    wait_frames = int(waiting_s / DT_CTRL)
    if (self.sm.frame - self._mpc_event_frame) < max(wait_frames, 1):
      return
    try:
      evt = EventName(int(mpc_evt))
    except Exception:
      return
    self.events.add(evt)
    self._mpc_event_frame = self.sm.frame
    self._mpc_event_prev = int(mpc_evt)

  def update_events(self, CS):
    self.events.clear()

    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    if not self.initialized:
      self.events.add(EventName.controlsInitializing)
      return

    if CS.gasPressed:
      self.events.add(EventName.pedalPressedPreEnable if self.disengage_on_accelerator else
                      EventName.gasPressedOverride)

    self.events.add_from_msg(CS.events)

    if not self.CP.notCar:
      self.events.add_from_msg(self.sm['driverMonitoringState'].events)

    # ✅ commaai#26472: Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and (not self.v_cruise_helper.v_cruise_initialized) and resume_pressed:
      self.events.add(EventName.resumeBlocked)

    if EON and (self.sm['peripheralState'].pandaType != PandaType.uno) and \
       self.sm['deviceState'].batteryPercent < 1 and self.sm['deviceState'].chargingError \
       and not self.params.get_bool("IsChargerFaultIgnored"):
      self.events.add(EventName.lowBattery)
    if self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
      self.events.add(EventName.outOfSpace)
    if self.sm['deviceState'].memoryUsagePercent > (90 if TICI else 65) and not SIMULATION:
      self.events.add(EventName.lowMemory)

    if self.sm['peripheralState'].pandaType in (PandaType.uno, PandaType.dos):
      if self.sm['peripheralState'].fanSpeedRpm == 0 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 5.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = self.sm.frame

    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != log.LiveCalibrationData.Status.calibrated:
      if cal_status == log.LiveCalibrationData.Status.uncalibrated:
        self.events.add(EventName.calibrationIncomplete)
      elif cal_status == log.LiveCalibrationData.Status.recalibrating:
        self.events.add(EventName.calibrationRecalibrating)
      else:
        self.events.add(EventName.calibrationInvalid)

    if self.sm['lateralPlan'].laneChangeState == LaneChangeState.preLaneChange:
      direction = self.sm['lateralPlan'].laneChangeDirection
      left_road_edge = -self.sm['modelV2'].roadEdges[0].y[0]
      right_road_edge = self.sm['modelV2'].roadEdges[1].y[0]

      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)
      elif ((left_road_edge < 3.5) and direction == LaneChangeDirection.left) or \
           ((right_road_edge < 3.5) and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)
      else:
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
        else:
          self.events.add(EventName.preLaneChangeRight)
    elif self.sm['lateralPlan'].laneChangeState in (LaneChangeState.laneChangeStarting,
                                                   LaneChangeState.laneChangeFinishing):
      self.events.add(EventName.laneChange)

    if CS.canTimeout:
      self.events.add(EventName.canBusMissing)
    elif not CS.canValid:
      self.events.add(EventName.canError)

    for i, pandaState in enumerate(self.sm['pandaStates']):
      if i < len(self.CP.safetyConfigs):
        safety_mismatch = pandaState.safetyModel != self.CP.safetyConfigs[i].safetyModel or \
                          pandaState.safetyParam != self.CP.safetyConfigs[i].safetyParam or \
                          pandaState.alternativeExperience != self.CP.alternativeExperience
      else:
        safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

      if safety_mismatch or self.mismatch_counter >= 200:
        self.events.add(EventName.controlsMismatch)

      if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
        self.events.add(EventName.relayMalfunction)

    if len(self.sm['radarState'].radarErrors):
      self.events.add(EventName.radarFault)
    elif not self.sm.valid["pandaStates"]:
      self.events.add(EventName.usbError)
    elif not self.sm.all_checks() or self.can_rcv_error:
      if not self.sm.all_alive():
        self.events.add(EventName.commIssue)
      elif not self.sm.all_freq_ok():
        self.events.add(EventName.commIssueAvgFreq)
      else:
        self.events.add(EventName.commIssue)

      if not self.logged_comm_issue:
        invalid = [s for s, valid in self.sm.valid.items() if not valid]
        not_alive = [s for s, alive in self.sm.alive.items() if not alive]
        not_freq_ok = [s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok]
        cloudlog.event("commIssue", invalid=invalid, not_alive=not_alive, not_freq_ok=not_freq_ok,
                       can_error=self.can_rcv_error, error=True)
        self.logged_comm_issue = True
    else:
      self.logged_comm_issue = False

    if not self.sm['liveParameters'].valid:
      self.events.add(EventName.vehicleModelInvalid)
    if not self.sm['lateralPlan'].mpcSolutionValid and not (EventName.turningIndicatorOn in self.events.names):
      self.events.add(EventName.plannerError)
    if not (self.sm['liveParameters'].sensorValid or self.sm['liveLocationKalman'].sensorsOK) and not NOSENSOR:
      if self.sm.frame > 5 / DT_CTRL:
        self.events.add(EventName.sensorDataInvalid)
    if not self.sm['liveLocationKalman'].posenetOK:
      self.events.add(EventName.posenetInvalid)
    if not self.sm['liveLocationKalman'].deviceStable:
      self.events.add(EventName.deviceFalling)

    if not REPLAY:
      cruise_mismatch = CS.cruiseState.enabledAcc and (not self.enabled or not self.CP.pcmCruise)
      self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
      if self.cruise_mismatch_counter > int(6. / DT_CTRL):
        self.events.add(EventName.cruiseMismatch)

    stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
    model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
    planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
    if not self.disable_op_fcw and (planner_fcw or model_fcw):
      self.events.add(EventName.fcw)

    # ===== MPC event -> UI Event (CruiseHelper 이식: 변경 감지 + 디바운스) =====
    try:
      mpc_evt = int(self.sm['longitudinalPlan'].mpcEvent)
    except Exception:
      mpc_evt = 0

    if self.enabled and self.CP.openpilotLongitudinalControl:
      if mpc_evt > 0 and mpc_evt != self._mpc_event_prev:
        self._send_mpc_event(mpc_evt, waiting_s=5.0)
    else:
      self._mpc_event_prev = 0
      self._mpc_event_frame = 0

    if TICI:
      for m in messaging.drain_sock(self.log_sock, wait_for_one=False):
        try:
          msg = m.androidLog.message
          if any(err in msg for err in ("ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED")):
            csid = msg.split("CSID:")[-1].split(" ")[0]
            evt = CSID_MAP.get(csid, None)
            if evt is not None:
              self.events.add(evt)
        except UnicodeDecodeError:
          pass

    if not SIMULATION:
      if not self.sm.all_alive(self.camera_packets):
        self.events.add(EventName.cameraMalfunction)
      elif not self.sm.all_freq_ok(self.camera_packets):
        self.events.add(EventName.cameraFrameRate)

      if self.sm['modelV2'].frameDropPerc > 20:
        self.events.add(EventName.modeldLagging)
      if self.sm['liveLocationKalman'].excessiveResets:
        self.events.add(EventName.localizerMalfunction)

      not_running = {p.name for p in self.sm['managerState'].processes if not p.running and p.shouldBeRunning}
      if self.sm.rcv_frame['managerState'] and (not_running - IGNORE_PROCESSES):
        self.events.add(EventName.processNotRunning)

    if not self.nn_alert_shown and self.sm.frame * DT_CTRL == 5.5 and self.CP.lateralTuning.which() == 'torque' and self.CI.use_nnff:
      self.nn_alert_shown = True
      self.events.add(EventName.torqueNNLoad)

  def data_sample(self):
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS = self.CI.update(self.CC, can_strs)

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      if all_valid or self.sm.frame * DT_CTRL > 3.5 or SIMULATION:
        if not self.read_only:
          self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])
        self.initialized = True

        if REPLAY and self.sm['pandaStates'][0].controlsAllowed:
          self.state = State.enabled

        self.params.put_bool("ControlsReady", True)

    if not can_strs:
      self.can_rcv_error_counter += 1
      self.can_rcv_error = True
    else:
      self.can_rcv_error = False

    if not self.enabled:
      self.mismatch_counter = 0

    if self.enabled and any(not ps.controlsAllowed for ps in self.sm['pandaStates']
                            if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.mismatch_counter += 1

    self.distance_traveled += CS.vEgo * DT_CTRL
    return CS

  def state_transition(self, CS):
    # keep in sync (CI 내부 CP 갱신 가능)
    self.CP.pcmCruise = self.CI.CP.pcmCruise

    # ✅ commaai#26472: helper가 v_cruise/v_cruise_cluster 관리
    self.v_cruise_helper.update_v_cruise(CS, self.enabled, self.is_metric)

    # SCC smoother
    SccSmoother.update_cruise_buttons(self, CS, self.CP.openpilotLongitudinalControl)

    # ✅✅ myDrivingMode / mySafeModeFactor sanitize
    try:
      self.myDrivingMode = int(self.myDrivingMode)
    except Exception:
      self.myDrivingMode = 0

    try:
      self.mySafeModeFactor = float(self.mySafeModeFactor)
    except Exception:
      self.mySafeModeFactor = 1.0

    self.mySafeModeFactor = float(clip(self.mySafeModeFactor, 0.1, 1.0))

    self.longCruiseGap = clip(int(self.sm['longitudinalPlan'].cruiseGap), 1, 4)
    lead = self.sm['radarState'].leadOne
    if lead.status:
      self.dRel = float(lead.dRel)
      self.vRel = float(lead.vRel)
    else:
      self.dRel = 0.0
      self.vRel = 0.0

    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)
    self.current_alert_types = [ET.PERMANENT]

    if self.state != State.disabled:
      if self.events.any(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)
      elif self.events.any(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)
      else:
        if self.state == State.enabled:
          if self.events.any(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(0.5 / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)
          elif self.events.any(ET.OVERRIDE):
            self.state = State.overriding
            self.current_alert_types.append(ET.OVERRIDE)
        elif self.state == State.softDisabling:
          if not self.events.any(ET.SOFT_DISABLE):
            self.state = State.enabled
          elif self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)
          elif self.soft_disable_timer <= 0:
            self.state = State.disabled
        elif self.state == State.preEnabled:
          if self.events.any(ET.NO_ENTRY):
            self.state = State.disabled
            self.current_alert_types.append(ET.NO_ENTRY)
          elif not self.events.any(ET.PRE_ENABLE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)
        elif self.state == State.overriding:
          if self.events.any(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)
          elif not self.events.any(ET.OVERRIDE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.OVERRIDE)

    elif self.state == State.disabled:
      if self.events.any(ET.ENABLE):
        if self.events.any(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)
        else:
          if self.events.any(ET.PRE_ENABLE):
            self.state = State.preEnabled
          elif self.events.any(ET.OVERRIDE):
            self.state = State.overriding
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)

          # ✅ commaai#26472: enable 시 helper 초기 set speed
          self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode)

    self.enabled = self.state in ENABLED_STATES
    self.active = self.state in ACTIVE_STATES
    if self.active:
      self.current_alert_types.append(ET.WARNING)

  def state_control(self, CS):
    params = self.sm['liveParameters']
    x = max(params.stiffnessFactor, 0.1)

    if ntune_common_enabled('useLiveSteerRatio'):
      sr = max(params.steerRatio, 0.1)
    else:
      sr = max(ntune_common_get('steerRatio'), 0.1)

    if self.params.get_bool('Steer_SRTune'):
      sr_v = float(int(self.params.get("Steer_SRTune_v", encoding="utf8"))) * 0.01
      sr = interp(CS.vEgo * 3.6, SR_SCALE_BP, SR_SCALE_V) * sr_v

    self.VM.update_params(x, sr)

    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered,
                                           torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    lat_plan = self.sm['lateralPlan']
    long_plan = self.sm['longitudinalPlan']

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled

    standstill = CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    CC.latActive = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.joystick_mode) \
                   and abs(CS.steeringAngleDeg) < self.CP.maxSteeringAngleDeg
    CC.longActive = self.active and not self.events.any(ET.OVERRIDE) and self.CP.openpilotLongitudinalControl

    hudControl = CC.hudControl
    xState = long_plan.xState
    hudControl.softHold = True if (xState == XState.softHold and CC.longActive) else False

    actuators = CC.actuators
    actuators.jerk = 0.0
    actuators.speed = 0.0

    self.long_control_state = self.LoC.long_control_state
    actuators.longControlState = self.long_control_state

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset(v_pid=CS.vEgo)
      self.long_control_state = LongControlState.off
      self.stopping = False

    if not CS.cruiseState.enabledAcc:
      self.LoC.reset(v_pid=CS.vEgo)

    if not self.joystick_mode:
      # ✅ helper 기준 set speed 사용
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo,
                                                      self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MS)
      t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL

      actuators.accel, actuators.jerk = self.LoC.update(CC.longActive and CS.cruiseState.enabledAcc,
                                                        CS, long_plan, pid_accel_limits, t_since_plan)

      self.long_control_state = actuators.longControlState
      self.stopping = (actuators.longControlState == LongControlState.stopping)

      if len(long_plan.speeds):
        actuators.speed = long_plan.speeds[-1]

      self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(
        self.CP, CS.vEgo,
        lat_plan.psis,
        lat_plan.curvatures,
        lat_plan.curvatureRates,
        long_plan.distances,
        self.average_desired_curvature
      )
      actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(
        CC.latActive, CS, self.VM, params,
        self.last_actuators, self.steer_limited, self.desired_curvature,
        self.desired_curvature_rate, self.sm['liveLocationKalman'],
        model_data=self.sm['modelV2']
      )
    else:
      lac_log = log.ControlsState.LateralDebugState.new_message()
      if self.sm.rcv_frame['testJoystick'] > 0:
        if CC.longActive:
          actuators.accel = 4.0 * clip(self.sm['testJoystick'].axes[0], -1, 1)
          self.long_control_state = actuators.longControlState
          self.stopping = (actuators.longControlState == LongControlState.stopping)

        if CC.latActive:
          steer = clip(self.sm['testJoystick'].axes[1], -1, 1)
          actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.

        lac_log.active = self.active
        lac_log.steeringAngleDeg = CS.steeringAngleDeg
        lac_log.output = actuators.steer
        lac_log.saturated = abs(actuators.steer) >= 0.9

    if lac_log.active and lac_log.saturated:
      dpath_points = lat_plan.dPathPoints
      if len(dpath_points):
        left_deviation = actuators.steer > 0 and dpath_points[0] < -0.20
        right_deviation = actuators.steer < 0 and dpath_points[0] > 0.20
        if left_deviation or right_deviation:
          self.events.add(EventName.steerSaturated)

    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue
      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish_logs(self, CS, start_time, CC, lac_log):
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
    # ✅ helper 기준 HUD setSpeed
    hudControl.setSpeed = float(self.v_cruise_helper.v_cruise_cluster_kph * CV.KPH_TO_MS)
    hudControl.speedVisible = self.enabled
    hudControl.lanesVisible = self.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead

    xState = self.sm['longitudinalPlan'].xState
    hudControl.softHold = True if (xState == XState.softHold and CC.longActive) else False

    hudControl.cruiseGap = clip(int(self.sm['longitudinalPlan'].cruiseGap), 1, 4)
    hudControl.objDist = int(self.dRel)
    hudControl.objRelSpd = float(self.vRel)

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

    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, CC, self.sm, self.is_metric, self.soft_disable_timer])
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

    # ✅ applyMaxSpeed 유지 + fallback은 helper 기준
    controlsState.vCruise = float(self.applyMaxSpeed if self.CP.openpilotLongitudinalControl
                                  else self.v_cruise_helper.v_cruise_kph)

    controlsState.upAccelCmd = float(self.LoC.pid.p)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    controlsState.startMonoTime = int(start_time * 1e9)
    controlsState.forceDecel = bool(force_decel)
    controlsState.canErrorCounter = self.can_rcv_error_counter
    controlsState.distanceTraveled = self.distance_traveled
    controlsState.experimentalMode = self.experimental_mode

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

    # ✅✅ planner에서 읽는 필드 publish
    controlsState.myDrivingMode = int(self.myDrivingMode)
    controlsState.mySafeModeFactor = float(self.mySafeModeFactor)

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

    # carState
    car_events = self.events.to_msg()
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.events = car_events
    self.pm.send('carState', cs_send)

    # carEvents
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('carEvents', len(self.events))
      ce_send.carEvents = car_events
      self.pm.send('carEvents', ce_send)
    self.events_prev = self.events.names.copy()

    # carParams - logged every 50 seconds
    if (self.sm.frame % int(50. / DT_CTRL) == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

    # copy CarControl
    self.CC = CC

  def step(self):
    start_time = sec_since_boot()
    self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl

    # Sample data
    CS = self.data_sample()

    # Update events & transitions
    self.update_events(CS)
    if not self.read_only and self.initialized:
      self.state_transition(CS)

    # Control
    CC, lac_log = self.state_control(CS)

    # Publish
    self.publish_logs(CS, start_time, CC, lac_log)

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
