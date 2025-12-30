#!/usr/bin/env python3
import os
from enum import IntEnum
from typing import Dict, Union, Callable, List, Optional

from common.params import Params
from cereal import log, car
import cereal.messaging as messaging
from common.conversions import Conversions as CV

from common.realtime import DT_CTRL
from selfdrive.locationd.calibrationd import MIN_SPEED_FILTER
from selfdrive.version import get_short_branch

AlertSize = log.ControlsState.AlertSize
AlertStatus = log.ControlsState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = car.CarEvent.EventName


# Alert priorities
class Priority(IntEnum):
  LOWEST = 0
  LOWER = 1
  LOW = 2
  MID = 3
  HIGH = 4
  HIGHEST = 5


# Event types
class ET:
  ENABLE = 'enable'
  PRE_ENABLE = 'preEnable'
  OVERRIDE = 'override'
  NO_ENTRY = 'noEntry'
  WARNING = 'warning'
  USER_DISABLE = 'userDisable'
  SOFT_DISABLE = 'softDisable'
  IMMEDIATE_DISABLE = 'immediateDisable'
  PERMANENT = 'permanent'


# get event name from enum
EVENT_NAME = {v: k for k, v in EventName.schema.enumerants.items()}


class Events:
  def __init__(self):
    self.events: List[int] = []
    self.static_events: List[int] = []
    # ✅ 안전: EVENTS에 없는 이벤트가 들어와도 터지지 않게 기본 dict로 시작
    self.events_prev: Dict[int, int] = {}

  @property
  def names(self) -> List[int]:
    return self.events

  def __len__(self) -> int:
    return len(self.events)

  def add(self, event_name: int, static: bool = False) -> None:
    # ✅ 안전: 새 이벤트가 들어오면 events_prev에 자동 등록
    if event_name not in self.events_prev:
      self.events_prev[event_name] = 0

    if static:
      self.static_events.append(event_name)
    self.events.append(event_name)

  def clear(self) -> None:
    # ✅ 안전: 기존 키는 카운트 업데이트, 이번 프레임 새로 들어온 이벤트도 누락 없이 반영
    new_prev: Dict[int, int] = {}
    # 기존 키 업데이트
    for k, v in self.events_prev.items():
      new_prev[k] = (v + 1) if (k in self.events) else 0
    # 이번 프레임에만 등장한(기존에 없던) 이벤트도 등록
    for e in self.events:
      if e not in new_prev:
        new_prev[e] = 0

    self.events_prev = new_prev
    self.events = self.static_events.copy()

  def any(self, event_type: str) -> bool:
    return any(event_type in EVENTS.get(e, {}) for e in self.events)

  def create_alerts(self, event_types: List[str], callback_args=None):
    """
    callback_args is expected:
      [CP, CC, sm, metric, soft_disable_time]
    (구형 호환을 위해 4개짜리 콜백도 자동 지원)
    """
    if callback_args is None:
      callback_args = []

    ret = []
    for e in self.events:
      # ✅ 핵심: EVENTS[e] 직접 접근 금지 (없으면 KeyError)
      ev_map = EVENTS.get(e, {})
      if not ev_map:
        continue

      types = ev_map.keys()
      for et in event_types:
        if et in types:
          alert = ev_map[et]
          if not isinstance(alert, Alert):
            # ✅ 5개 우선 + 4개 구형 콜백 호환
            try:
              alert = alert(*callback_args)
            except TypeError:
              alert = alert(*callback_args[:4])

          # ✅ events_prev에도 없을 수 있으니 get 사용
          prev = self.events_prev.get(e, 0)
          if DT_CTRL * (prev + 1) >= alert.creation_delay:
            alert.alert_type = f"{EVENT_NAME.get(e, str(e))}/{et}"
            alert.event_type = et
            ret.append(alert)
    return ret

  def add_from_msg(self, events):
    for e in events:
      self.add(e.name.raw)

  def to_msg(self):
    ret = []
    for event_name in self.events:
      event = car.CarEvent.new_message()
      event.name = event_name
      for event_type in EVENTS.get(event_name, {}):
        setattr(event, event_type, True)
      ret.append(event)
    return ret


class Alert:
  def __init__(self,
               alert_text_1: str,
               alert_text_2: str,
               alert_status: log.ControlsState.AlertStatus,
               alert_size: log.ControlsState.AlertSize,
               priority: Priority,
               visual_alert: car.CarControl.HUDControl.VisualAlert,
               audible_alert: car.CarControl.HUDControl.AudibleAlert,
               duration: float,
               alert_rate: float = 0.,
               creation_delay: float = 0.):

    self.alert_text_1 = alert_text_1
    self.alert_text_2 = alert_text_2
    self.alert_status = alert_status
    self.alert_size = alert_size
    self.priority = priority
    self.visual_alert = visual_alert
    self.audible_alert = audible_alert

    self.duration = int(duration / DT_CTRL)

    self.alert_rate = alert_rate
    self.creation_delay = creation_delay

    self.alert_type = ""
    self.event_type: Optional[str] = None

  def __str__(self) -> str:
    return f"{self.alert_text_1}/{self.alert_text_2} {self.priority} {self.visual_alert} {self.audible_alert}"

  def __gt__(self, alert2) -> bool:
    return self.priority > alert2.priority


class NoEntryAlert(Alert):
  def __init__(self, alert_text_2: str, visual_alert: car.CarControl.HUDControl.VisualAlert = VisualAlert.none):
    super().__init__("오픈파일럿 사용불가", alert_text_2, AlertStatus.normal,
                     AlertSize.mid, Priority.LOW, visual_alert,
                     AudibleAlert.refuse, 3.)


class SoftDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__("", alert_text_2,
                     AlertStatus.userPrompt, AlertSize.full,
                     Priority.MID, VisualAlert.steerRequired,
                     AudibleAlert.none, 0.)


# less harsh version of SoftDisable, where the condition is user-triggered
class UserSoftDisableAlert(SoftDisableAlert):
  def __init__(self, alert_text_2: str):
    super().__init__(alert_text_2)
    self.alert_text_1 = "openpilot will disengage"


class ImmediateDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__("핸들을 즉시 잡아주세요", alert_text_2,
                     AlertStatus.critical, AlertSize.full,
                     Priority.HIGHEST, VisualAlert.steerRequired,
                     AudibleAlert.warningSoft, 2.)


class EngagementAlert(Alert):
  def __init__(self, audible_alert: car.CarControl.HUDControl.AudibleAlert):
    super().__init__("", "",
                     AlertStatus.normal, AlertSize.none,
                     Priority.MID, VisualAlert.none,
                     audible_alert, .2)


class NormalPermanentAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "", duration: float = 0.2,
               priority: Priority = Priority.LOWER, creation_delay: float = 0.):
    super().__init__(alert_text_1, alert_text_2,
                     AlertStatus.normal, AlertSize.mid if len(alert_text_2) else AlertSize.small,
                     priority, VisualAlert.none, AudibleAlert.none, duration, creation_delay=creation_delay)


class StartupAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "The  Dynamic  Luxury  Driving",
               alert_status=AlertStatus.normal):
    super().__init__(alert_text_1, alert_text_2,
                     alert_status, AlertSize.mid,
                     Priority.LOWER, VisualAlert.none, AudibleAlert.none, 5.)


# ********** helper functions **********
def get_display_speed(speed_ms: float, metric: bool) -> str:
  speed = int(round(speed_ms * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)))
  unit = 'km/h' if metric else 'mph'
  return f"{speed} {unit}"


# ********** alert callback functions **********
AlertCallbackType = Callable[[car.CarParams, car.CarControl, messaging.SubMaster, bool, int], Alert]


def soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    return SoftDisableAlert(alert_text_2)
  return func


def user_soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    return UserSoftDisableAlert(alert_text_2)
  return func


def startup_master_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  branch = get_short_branch("")
  if "REPLAY" in os.environ:
    branch = "replay"
  return StartupAlert("WARNING: This branch is not tested", branch, alert_status=AlertStatus.userPrompt)


def below_engage_speed_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NoEntryAlert(f"Speed Below {get_display_speed(CP.minEnableSpeed, metric)}")


def below_steer_speed_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    f"Steer Unavailable Below {get_display_speed(CP.minSteerSpeed, metric)}",
    "",
    AlertStatus.userPrompt, AlertSize.small,
    Priority.MID, VisualAlert.steerRequired, AudibleAlert.prompt, 0.4)


def calibration_incomplete_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  first_word = 'Recalibration' if sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.recalibrating else 'Calibration'
  return Alert(
    f"{first_word} in Progress: {sm['liveCalibration'].calPerc:.0f}%",
    f"주행속도 {get_display_speed(MIN_SPEED_FILTER, metric)} 이상 주행!",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2)


def torque_nn_load_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  model_name = Params().get("NNFFModelName", encoding='utf-8')
  if model_name == "":
    return Alert(
      "NNFF Torque Controller not available",
      "Donate logs to Twilsonco to get it added!",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 6.0)
  else:
    return Alert(
      "NNFF Torque Controller loaded",
      model_name,
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.engage, 5.0)


def no_gps_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  gps_integrated = sm['peripheralState'].pandaType in (log.PandaState.PandaType.uno, log.PandaState.PandaType.dos)
  return Alert(
    "Poor GPS reception",
    "Hardware malfunctioning if sky is visible" if gps_integrated else "Check GPS antenna placement",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=300.)


def low_memory_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NormalPermanentAlert("Low Memory", f"{sm['deviceState'].memoryUsagePercent}% used")


def wrong_car_mode_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  text = "Cruise Mode Disabled"
  if CP.carName == "honda":
    text = "Main Switch Off"
  return NoEntryAlert(text)


def joystick_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  axes = sm['testJoystick'].axes
  gb, steer = list(axes)[:2] if len(axes) else (0., 0.)
  vals = f"Gas: {round(gb * 100.)}%, Steer: {round(steer * 100.)}%"
  return NormalPermanentAlert("Joystick Mode", vals)


def curve_speed_adjust_alert(CP: car.CarParams, _CC: car.CarControl, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  speedLimit = sm['longitudinalPlan'].visionTurnSpeed
  speed = round(speedLimit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
  message = f'Adjusting to {speed} {"km/h" if metric else "mph"} curve speed'
  return Alert(
    message,
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, 4.)


EVENTS: Dict[int, Dict[str, Union[Alert, AlertCallbackType]]] = {
  # ********** events with no alerts **********
  EventName.stockFcw: {},

  # ********** events only containing alerts displayed in all states **********
  EventName.joystickDebug: {
    ET.WARNING: joystick_alert,
    ET.PERMANENT: NormalPermanentAlert("Joystick Mode"),
  },

  EventName.controlsInitializing: {
    ET.NO_ENTRY: NoEntryAlert("System Initializing"),
  },

  EventName.startup: {
    ET.PERMANENT: StartupAlert("G  E  N  E  S  I  S")
  },

  EventName.startupMaster: {
    ET.PERMANENT: startup_master_alert,
  },

  # Car is recognized, but marked as dashcam only
  EventName.startupNoControl: {
    ET.PERMANENT: StartupAlert("Dashcam mode"),
  },

  # Car is not recognized
  EventName.startupNoCar: {
    ET.PERMANENT: StartupAlert("Dashcam mode for unsupported car"),
  },

  EventName.startupNoFw: {
    ET.PERMANENT: StartupAlert("Car Unrecognized",
                               "Check comma power connections",
                               alert_status=AlertStatus.userPrompt),
  },

  EventName.dashcamMode: {
    ET.PERMANENT: NormalPermanentAlert("Dashcam Mode",
                                       priority=Priority.LOWEST),
  },

  EventName.invalidLkasSetting: {
    ET.PERMANENT: NormalPermanentAlert("Stock LKAS is on",
                                       "Turn off stock LKAS to engage"),
  },

  EventName.cruiseMismatch: {
    # ET.PERMANENT: ImmediateDisableAlert("openpilot failed to cancel cruise"),
  },

  EventName.carUnrecognized: {
    ET.PERMANENT: NormalPermanentAlert("Dashcam Mode",
                                       "Car Unrecognized",
                                       priority=Priority.LOWEST),
  },

  EventName.stockAeb: {
    ET.PERMANENT: Alert(
      "브레이크!",
      "추돌위험",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
    ET.NO_ENTRY: NoEntryAlert("Stock AEB: Risk of Collision"),
  },

  EventName.fcw: {
    ET.PERMANENT: Alert(
      "브레이크!",
      "추돌위험",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
  },

  EventName.ldw: {
    ET.PERMANENT: Alert(
      "차선이탈 감지",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.ldw, AudibleAlert.prompt, 3.),
  },

  # ********** events only containing alerts that display while engaged **********
  EventName.vehicleModelInvalid: {
    ET.NO_ENTRY: NoEntryAlert("Vehicle Parameter Identification Failed"),
    ET.SOFT_DISABLE: soft_disable_alert("Vehicle Parameter Identification Failed"),
  },

  EventName.steerTempUnavailableSilent: {
    ET.WARNING: Alert(
      "조향제어 일시적 사용불가",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, 1.),
  },

  EventName.preDriverDistracted: {
    ET.WARNING: Alert(
      "Pay Attention",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.promptDriverDistracted: {
    ET.WARNING: Alert(
      "Pay Attention",
      "Driver Distracted",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverDistracted: {
    ET.WARNING: Alert(
      "조향제어가 강제로 해제됩니다",
      "운전자 도로주시 불안",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.preDriverUnresponsive: {
    ET.WARNING: Alert(
      "Touch Steering Wheel: No Face Detected",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.promptDriverUnresponsive: {
    ET.WARNING: Alert(
      "Touch Steering Wheel",
      "Driver Unresponsive",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },

  EventName.driverUnresponsive: {
    ET.WARNING: Alert(
      "DISENGAGE IMMEDIATELY",
      "Driver Unresponsive",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.manualRestart: {
    ET.WARNING: Alert(
      "TAKE CONTROL",
      "Resume Driving Manually",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.resumeRequired: {
    ET.WARNING: Alert(
      "전방차량 멈춤",
      "차량 출발시 자동 재출발",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.belowSteerSpeed: {
    ET.WARNING: below_steer_speed_alert,
  },

  EventName.preLaneChangeLeft: {
    ET.WARNING: Alert(
      "좌측 차선 변경",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.preLaneChangeRight: {
    ET.WARNING: Alert(
      "우측 차선 변경",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },

  EventName.laneChangeBlocked: {
    ET.WARNING: Alert(
      "Road Edge or 후측방 작동중",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.laneChange: {
    ET.WARNING: Alert(
      "Auto Lane Change",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.steerSaturated: {
    ET.WARNING: Alert(
      "핸들을 잡아주세요",
      "조향제어 제한을 초과함",
      AlertStatus.userPrompt, AlertSize.none,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventName.fanMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Fan Malfunction", "Likely Hardware Issue"),
  },

  EventName.cameraMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Camera Malfunction", "Likely Hardware Issue"),
  },

  EventName.cameraFrameRate: {
    ET.PERMANENT: NormalPermanentAlert("Camera Frame Rate Low", "Reboot your Device"),
  },

  EventName.gpsMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("GPS Malfunction", "Likely Hardware Issue"),
  },

  EventName.localizerMalfunction: {
    # ET.PERMANENT: NormalPermanentAlert("Sensor Malfunction", "Hardware Malfunction"),
  },

  EventName.visionEntering: {
    ET.WARNING: Alert(
      "Curve Entering",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  EventName.visionTurning: {
    ET.WARNING: Alert(
      "Curve Turning",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  EventName.visionleaving: {
    ET.WARNING: Alert(
      "Curve Leaving",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  EventName.curvespeedValueChange: {
    ET.WARNING: curve_speed_adjust_alert,
  },

  # ********** events that affect controls state transitions **********
  EventName.pcmEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.buttonEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventName.pcmDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventName.buttonCancel: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventName.brakeHold: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Active"),
  },

  EventName.parkBrake: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Parking Brake Engaged"),
  },

  EventName.pedalPressed: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Pedal Pressed",
                              visual_alert=VisualAlert.brakePressed),
  },

  EventName.pedalPressedPreEnable: {
    ET.PRE_ENABLE: Alert(
      "Release Pedal to Engage",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1, creation_delay=1.),
  },

  EventName.gasPressedOverride: {
    ET.OVERRIDE: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.wrongCarMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: wrong_car_mode_alert,
  },

  EventName.resumeBlocked: {
    ET.NO_ENTRY: NoEntryAlert("Press Set to Engage"),
  },

  EventName.wrongCruiseMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Adaptive Cruise Disabled"),
  },

  EventName.steerTempUnavailable: {
    ET.SOFT_DISABLE: soft_disable_alert("Steering Temporarily Unavailable"),
    ET.NO_ENTRY: NoEntryAlert("Steering Temporarily Unavailable"),
  },

  EventName.outOfSpace: {
    ET.PERMANENT: NormalPermanentAlert("Out of Storage"),
    ET.NO_ENTRY: NoEntryAlert("Out of Storage"),
  },

  EventName.belowEngageSpeed: {
    ET.NO_ENTRY: below_engage_speed_alert,
  },

  EventName.sensorDataInvalid: {
    ET.PERMANENT: Alert(
      "No Data from Device Sensors",
      "Reboot your Device",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("No Data from Device Sensors"),
  },

  EventName.noGps: {
    ET.PERMANENT: no_gps_alert,
  },

  EventName.soundsUnavailable: {
    ET.PERMANENT: NormalPermanentAlert("Speaker not found", "Reboot your Device"),
    ET.NO_ENTRY: NoEntryAlert("Speaker not found"),
  },

  EventName.tooDistracted: {
    ET.NO_ENTRY: NoEntryAlert("Distraction Level Too High"),
  },

  EventName.overheat: {
    ET.PERMANENT: NormalPermanentAlert("System Overheated"),
    ET.SOFT_DISABLE: soft_disable_alert("System Overheated"),
    ET.NO_ENTRY: NoEntryAlert("System Overheated"),
  },

  EventName.wrongGear: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Gear not D"),
    ET.NO_ENTRY: NoEntryAlert("Gear not D"),
  },

  EventName.calibrationInvalid: {
    ET.PERMANENT: NormalPermanentAlert("Calibration Invalid", "Remount Device and Recalibrate"),
    ET.SOFT_DISABLE: soft_disable_alert("Calibration Invalid: Remount Device & Recalibrate"),
    ET.NO_ENTRY: NoEntryAlert("Calibration Invalid: Remount Device & Recalibrate"),
  },

  EventName.calibrationIncomplete: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert("캘리브레이션 유효하지않음"),
    ET.NO_ENTRY: NoEntryAlert("캘리브레이션 진행중..."),
  },

  EventName.calibrationRecalibrating: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert("마운트위치 오류 확인: 재캘리브레이션 진행중..."),
    ET.NO_ENTRY: NoEntryAlert("마운트위치 오류 확인: 재캘리브레이션 진행중..."),
  },

  EventName.doorOpen: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Door Open"),
    ET.NO_ENTRY: NoEntryAlert("Door Open"),
  },

  EventName.seatbeltNotLatched: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Seatbelt Unlatched"),
    ET.NO_ENTRY: NoEntryAlert("Seatbelt Unlatched"),
  },

  EventName.espDisabled: {
    ET.SOFT_DISABLE: soft_disable_alert("ESP Off"),
    ET.NO_ENTRY: NoEntryAlert("ESP Off"),
  },

  EventName.lowBattery: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Battery"),
    ET.NO_ENTRY: NoEntryAlert("Low Battery"),
  },

  EventName.commIssue: {
    ET.SOFT_DISABLE: soft_disable_alert("Communication Issue between Processes"),
    ET.NO_ENTRY: NoEntryAlert("Communication Issue between Processes"),
  },

  EventName.commIssueAvgFreq: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Communication Rate between Processes"),
    ET.NO_ENTRY: NoEntryAlert("Low Communication Rate between Processes"),
  },

  EventName.processNotRunning: {
    ET.NO_ENTRY: NoEntryAlert("System Malfunction: Reboot Your Device"),
  },

  EventName.radarFault: {
    ET.SOFT_DISABLE: soft_disable_alert("Radar Error: Restart the Car"),
    ET.NO_ENTRY: NoEntryAlert("Radar Error: Restart the Car"),
  },

  EventName.modeldLagging: {
    ET.SOFT_DISABLE: soft_disable_alert("Driving model lagging"),
    ET.NO_ENTRY: NoEntryAlert("Driving model lagging"),
  },

  EventName.posenetInvalid: {
    ET.SOFT_DISABLE: soft_disable_alert("Model Output Uncertain"),
    ET.NO_ENTRY: NoEntryAlert("Model Output Uncertain"),
  },

  EventName.deviceFalling: {
    ET.SOFT_DISABLE: soft_disable_alert("Device Fell Off Mount"),
    ET.NO_ENTRY: NoEntryAlert("Device Fell Off Mount"),
  },

  EventName.lowMemory: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Memory: Reboot Your Device"),
    ET.PERMANENT: NormalPermanentAlert("Low Memory", "Reboot your Device"),
    ET.NO_ENTRY: NoEntryAlert("Low Memory: Reboot Your Device"),
  },

  EventName.highCpuUsage: {
    ET.NO_ENTRY: NoEntryAlert("System Malfunction: Reboot Your Device"),
  },

  EventName.accFaulted: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Faulted"),
    ET.PERMANENT: NormalPermanentAlert("Cruise Faulted", ""),
    ET.NO_ENTRY: NoEntryAlert("Cruise Faulted"),
  },

  EventName.controlsMismatch: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Controls Mismatch"),
    ET.NO_ENTRY: NoEntryAlert("Controls Mismatch"),
  },

  EventName.roadCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera CRC Error - Road",
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.wideRoadCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera CRC Error - Road Fisheye",
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.driverCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera CRC Error - Driver",
                                       duration=1.,
                                       creation_delay=30.),
  },

  EventName.usbError: {
    ET.SOFT_DISABLE: soft_disable_alert("USB Error: Reboot Your Device"),
    ET.PERMANENT: NormalPermanentAlert("USB Error: Reboot Your Device", ""),
    ET.NO_ENTRY: NoEntryAlert("USB Error: Reboot Your Device"),
  },

  EventName.canError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("CAN Error"),
    ET.PERMANENT: Alert(
      "CAN Error: Check Connections",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("CAN Error: Check Connections"),
  },

  EventName.canBusMissing: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("CAN Bus Disconnected"),
    ET.PERMANENT: Alert(
      "CAN Bus Disconnected: Likely Faulty Cable",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("CAN Bus Disconnected: Check Connections"),
  },

  EventName.steerUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("LKAS 오류 : 차량을 재가동하세요"),
    ET.PERMANENT: NormalPermanentAlert("LKAS 오류 : 차량을 재가동하세요"),
    ET.NO_ENTRY: NoEntryAlert("LKAS 오류 : 차량을 재가동하세요"),
  },

  EventName.brakeUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Fault: Restart the Car"),
    ET.PERMANENT: NormalPermanentAlert("Cruise Fault: Restart the car to engage"),
    ET.NO_ENTRY: NoEntryAlert("Cruise Fault: Restart the Car"),
  },

  EventName.reverseGear: {
    ET.PERMANENT: Alert(
      "GENESIS",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.SOFT_DISABLE: SoftDisableAlert("GENESIS"),
    ET.NO_ENTRY: NoEntryAlert("GENESIS"),
  },

  EventName.cruiseDisabled: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Is Off"),
  },

  EventName.plannerError: {
    ET.SOFT_DISABLE: SoftDisableAlert("Planner Solution Error"),
    ET.NO_ENTRY: NoEntryAlert("Planner Solution Error"),
  },

  EventName.relayMalfunction: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("하네스 오작동"),
    ET.PERMANENT: NormalPermanentAlert("하네스 오작동", "하드웨어를 점검하세요"),
    ET.NO_ENTRY: NoEntryAlert("하네스 오작동"),
  },

  EventName.noTarget: {
    ET.IMMEDIATE_DISABLE: Alert(
      "오픈파일럿 사용불가",
      "근접 앞차량이 없습니다",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 3.),
    ET.NO_ENTRY: NoEntryAlert("No Close Lead Car"),
  },

  EventName.speedTooLow: {
    ET.IMMEDIATE_DISABLE: Alert(
      "오픈파일럿 사용불가",
      "속도를 높이고 재가동하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 3.),
  },

  EventName.speedTooHigh: {
    ET.WARNING: Alert(
      "속도가 너무 높습니다",
      "속도를 줄여주세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 4.),
    ET.NO_ENTRY: NoEntryAlert("Slow down to engage"),
  },

  EventName.lowSpeedLockout: {
    ET.PERMANENT: NormalPermanentAlert("Cruise Fault: Restart the car to engage"),
    ET.NO_ENTRY: NoEntryAlert("Cruise Fault: Restart the Car"),
  },

  EventName.lkasDisabled: {
    ET.PERMANENT: NormalPermanentAlert("LKAS Disabled: Enable LKAS to engage"),
    ET.NO_ENTRY: NoEntryAlert("LKAS Disabled"),
  },

  EventName.turningIndicatorOn: {
    ET.WARNING: Alert(
      "조향 유지중...",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },

  EventName.torqueNNLoad: {
    ET.PERMANENT: torque_nn_load_alert,
  },

  # ==========================
  # ✅ 사용자 추가: Traffic Events
  # ==========================
  EventName.trafficStopping: {
    ET.WARNING: Alert(
      "신호 감지",
      "감속/정지 중입니다",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none,
      2.0,
      alert_rate=1.0),
  },

  EventName.trafficSignGreen: {
    ET.WARNING: Alert(
      "출발합니다",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none,
      2.0,
      alert_rate=0.5),
  },

  EventName.trafficSignChanged: {
    ET.WARNING: Alert(
      "신호가 바뀌었어요",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none,
      2.0,
      alert_rate=0.5),
  },

  EventName.slowingDownSpeed: {
    ET.PERMANENT: Alert("과속카메라 감지 : 감속중","", AlertStatus.normal, AlertSize.small,
      Priority.MID, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventName.slowingDownSpeedSound: {
    ET.PERMANENT: Alert("과속카메라 감지 : 감속중","", AlertStatus.normal, AlertSize.small,
      Priority.HIGH, VisualAlert.none, AudibleAlert.none, 2.),
  },
}
