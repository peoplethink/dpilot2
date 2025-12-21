using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0x8e2af1e708af8b8d;

# ******* events causing controls state machine transition *******

struct CarEvent @0x9b1657f34caf3ad3 {
  name @0 :EventName;

  # event types
  enable @1 :Bool;
  noEntry @2 :Bool;
  warning @3 :Bool;   # alerts presented only when enabled or soft disabling
  userDisable @4 :Bool;
  softDisable @5 :Bool;
  immediateDisable @6 :Bool;
  preEnable @7 :Bool;
  permanent @8 :Bool; # alerts presented regardless of openpilot state
  override @9 :Bool;

  enum EventName @0xbaa8c5d505f727de {
    canError @0;
    steerUnavailable @1;
    brakeUnavailable @2;
    wrongGear @4;
    doorOpen @5;
    seatbeltNotLatched @6;
    espDisabled @7;
    wrongCarMode @8;
    steerTempUnavailable @9;
    reverseGear @10;
    buttonCancel @11;
    buttonEnable @12;
    pedalPressed @13;  # exits active state
    pedalPressedPreEnable @73;  # added during pre-enable state for either pedal
    gasPressedOverride @108;  # added when user is pressing gas with no disengage on gas
    cruiseDisabled @14;
    speedTooLow @17;
    outOfSpace @18;
    overheat @19;
    calibrationIncomplete @20;
    calibrationInvalid @21;
    calibrationRecalibrating @122;
    torqueNNLoad @123;
    controlsMismatch @22;
    pcmEnable @23;
    pcmDisable @24;
    noTarget @25;
    radarFault @26;
    brakeHold @28;
    parkBrake @29;
    manualRestart @30;
    lowSpeedLockout @31;
    plannerError @32;
    joystickDebug @34;
    steerTempUnavailableSilent @35;
    resumeRequired @36;
    preDriverDistracted @37;
    promptDriverDistracted @38;
    driverDistracted @39;
    preDriverUnresponsive @43;
    promptDriverUnresponsive @44;
    driverUnresponsive @45;
    belowSteerSpeed @46;
    lowBattery @48;
    vehicleModelInvalid @50;
    accFaulted @51;
    sensorDataInvalid @52;
    commIssue @53;
    commIssueAvgFreq @109;
    tooDistracted @54;
    posenetInvalid @55;
    soundsUnavailable @56;
    preLaneChangeLeft @57;
    preLaneChangeRight @58;
    laneChange @59;
    lowMemory @63;
    stockAeb @64;
    ldw @65;
    carUnrecognized @66;
    invalidLkasSetting @69;
    speedTooHigh @70;
    laneChangeBlocked @71;
    relayMalfunction @72;
    stockFcw @74;
    startup @75;
    startupNoCar @76;
    startupNoControl @77;
    startupMaster @78;
    startupNoFw @104;
    fcw @79;
    steerSaturated @80;
    belowEngageSpeed @84;
    noGps @85;
    wrongCruiseMode @87;
    modeldLagging @89;
    deviceFalling @90;
    fanMalfunction @91;
    cameraMalfunction @92;
    cameraFrameRate @110;
    gpsMalfunction @94;
    processNotRunning @95;
    dashcamMode @96;
    controlsInitializing @98;
    usbError @99;
    roadCameraError @100;
    driverCameraError @101;
    wideRoadCameraError @102;
    localizerMalfunction @103;
    highCpuUsage @105;
    cruiseMismatch @106;
    lkasDisabled @107;
    canBusMissing @111;
    autoHold @124;

    radarCanErrorDEPRECATED @15;
    communityFeatureDisallowedDEPRECATED @62;
    radarCommIssueDEPRECATED @67;
    driverMonitorLowAccDEPRECATED @68;
    gasUnavailableDEPRECATED @3;
    dataNeededDEPRECATED @16;
    modelCommIssueDEPRECATED @27;
    ipasOverrideDEPRECATED @33;
    geofenceDEPRECATED @40;
    driverMonitorOnDEPRECATED @41;
    driverMonitorOffDEPRECATED @42;
    calibrationProgressDEPRECATED @47;
    invalidGiraffeHondaDEPRECATED @49;
    invalidGiraffeToyotaDEPRECATED @60;
    internetConnectivityNeededDEPRECATED @61;
    whitePandaUnsupportedDEPRECATED @81;
    commIssueWarningDEPRECATED @83;
    focusRecoverActiveDEPRECATED @86;
    neosUpdateRequiredDEPRECATED @88;
    modelLagWarningDEPRECATED @93;
    startupOneplusDEPRECATED @82;
    startupFuzzyFingerprintDEPRECATED @97;

    turningIndicatorOn @112;
    autoLaneChange @113;
    slowingDownSpeed @114;
    slowingDownSpeedSound @115;

    speedLimitActive @116;
    speedLimitValueChange @117;
    visionEntering @118;
    visionTurning @119;
    visionleaving @120;
    curvespeedValueChange @121;
    trafficStopping @124; #ajouatom
    trafficError @125; #ajouatom
    trafficSignGreen @126; #ajouatom
    trafficSignChanged @127; #ajouatom
  }
}

# ******* main car state @ 100hz *******
# all speeds in m/s

struct CarState {
  events @13 :List(CarEvent);

  # CAN health
  canValid @26 :Bool;       # invalid counter/checksums
  canTimeout @40 :Bool;     # CAN bus dropped out

  # car speed
  vEgo @1 :Float32;         # best estimate of speed
  aEgo @16 :Float32;        # best estimate of acceleration
  vEgoRaw @17 :Float32;     # unfiltered speed from CAN sensors
  yawRate @22 :Float32;     # best estimate of yaw rate
  standstill @18 :Bool;
  wheelSpeeds @2 :WheelSpeeds;
  vEgoCluster @51 :Float32;

  # gas pedal, 0.0-1.0
  gas @3 :Float32;        # this is user pedal only
  gasPressed @4 :Bool;    # this is user pedal only

  # brake pedal, 0.0-1.0
  brake @5 :Float32;      # this is user pedal only
  brakePressed @6 :Bool;  # this is user pedal only
  parkingBrake @39 :Bool;
  brakeHoldActive @38 :Bool;

  # steering wheel
  steeringAngleDeg @7 :Float32;
  steeringAngleOffsetDeg @37 :Float32; # Offset betweens sensors in case there multiple
  steeringRateDeg @15 :Float32;
  steeringTorque @8 :Float32;      # TODO: standardize units
  steeringTorqueEps @27 :Float32;  # TODO: standardize units
  steeringPressed @9 :Bool;        # if the user is using the steering wheel
  steerFaultTemporary @35 :Bool;   # temporary EPS fault
  steerFaultPermanent @36 :Bool;   # permanent EPS fault
  stockAeb @30 :Bool;
  stockFcw @31 :Bool;
  espDisabled @32 :Bool;

  # cruise state
  cruiseState @10 :CruiseState;

  # gear
  gearShifter @14 :GearShifter;

  # button presses
  buttonEvents @11 :List(ButtonEvent);
  leftBlinker @20 :Bool;
  rightBlinker @21 :Bool;
  genericToggle @23 :Bool;

  # lock info
  doorOpen @24 :Bool;
  seatbeltUnlatched @25 :Bool;

  # clutch (manual transmission only)
  clutchPressed @28 :Bool;

  # blindspot sensors
  leftBlindspot @33 :Bool; # Is there something blocking the left lane change
  rightBlindspot @34 :Bool; # Is there something blocking the right lane change

  cluSpeedMs @41 :Float32;

  cruiseGapDEPRECATED @42 :Int32;  # 기존 유지 (과거 호환용)
  autoHold @43 :Int32;
  tpms @44 :Tpms;
  vCluRatio @45 :Float32;
  aBasis @46 :Float32;
  currentGear @47 :Float32;

  cruiseGap @48 :Int32;

  engRpm @49 :Float32;
  radarDistance @50 :Float32;

  struct Tpms {
    fl @0 :Float32;
    fr @1 :Float32;
    rl @2 :Float32;
    rr @3 :Float32;
  }

  struct WheelSpeeds {
    fl @0 :Float32;
    fr @1 :Float32;
    rl @2 :Float32;
    rr @3 :Float32;
  }

  struct CruiseState {
    enabled @0 :Bool;
    speed @1 :Float32;
    available @2 :Bool;
    speedOffset @3 :Float32;
    standstill @4 :Bool;
    nonAdaptive @5 :Bool;
    enabledAcc @6 :Bool;
  }

  enum GearShifter {
    unknown @0;
    park @1;
    drive @2;
    neutral @3;
    reverse @4;
    sport @5;
    low @6;
    brake @7;
    eco @8;
    manumatic @9;
  }

  # send on change
  struct ButtonEvent {
    pressed @0 :Bool;
    type @1 :Type;

    enum Type {
      unknown @0;
      leftBlinker @1;
      rightBlinker @2;
      accelCruise @3;
      decelCruise @4;
      cancel @5;
      altButton1 @6;
      altButton2 @7;
      altButton3 @8;
      setCruise @9;
      resumeCruise @10;
      gapAdjustCruise @11;
    }
  }

  errorsDEPRECATED @0 :List(CarEvent.EventName);
  brakeLights @19 :Bool;
  steeringRateLimitedDEPRECATED @29 :Bool;
  canMonoTimesDEPRECATED @12: List(UInt64);
}

# ******* radar state @ 20hz *******

struct RadarData @0x888ad6581cf0aacb {
  errors @0 :List(Error);
  points @1 :List(RadarPoint);

  enum Error {
    canError @0;
    fault @1;
    wrongConfig @2;
  }

  struct RadarPoint {
    trackId @0 :UInt64;  # no trackId reuse

    dRel @1 :Float32; # m from the front bumper of the car
    yRel @2 :Float32; # m
    vRel @3 :Float32; # m/s

    aRel @4 :Float32; # m/s^2
    yvRel @5 :Float32; # m/s

    measured @6 :Bool;
  }

  canMonoTimesDEPRECATED @2 :List(UInt64);
}

# ******* car controls @ 100hz *******

struct CarControl {
  enabled @0 :Bool;
  latActive @11: Bool;
  longActive @12: Bool;

  actuators @6 :Actuators;

  actuatorsOutput @10 :Actuators;

  orientationNED @13 :List(Float32);
  angularVelocity @14 :List(Float32);

  cruiseControl @4 :CruiseControl;
  hudControl @5 :HUDControl;

  sccSmoother @15 :SccSmoother;

  struct SccSmoother {
    longControl @0: Bool;
    applyMaxSpeed @1 :Float32;
    cruiseMaxSpeed @2 :Float32;
    logMessage @3 :Text;
    autoTrGap @4 :UInt32;
  }

  struct Actuators {
    gas @0: Float32;
    brake @1: Float32;
    steer @2: Float32;
    steeringAngleDeg @3: Float32;

    speed @6: Float32; # m/s
    accel @4: Float32; # m/s^2
    longControlState @5: LongControlState;
    curvature @7: Float32;
    jerk @8: Float32;

    enum LongControlState @0xe40f3a917d908282{
      off @0;
      pid @1;
      stopping @2;
      starting @3;
    }
  }

  struct CruiseControl {
    cancel @0: Bool;
    resume @1: Bool;
    override @2: Bool;
    speedOverride @3: Float32;
    accelOverride @4: Float32;
  }

  struct HUDControl {
    speedVisible @0: Bool;
    setSpeed @1: Float32;
    lanesVisible @2: Bool;
    leadVisible @3: Bool;
    visualAlert @4: VisualAlert;
    audibleAlert @5: AudibleAlert;
    rightLaneVisible @6: Bool;
    leftLaneVisible @7: Bool;
    rightLaneDepart @8: Bool;
    leftLaneDepart @9: Bool;

    cruiseGap @10 :Int32;
    objDist @11 :Int32;
    objRelSpd @12 :Float32;
    softHold @13 :Bool;
    radarAlarm @14 :Bool;

    enum VisualAlert {
      none @0;
      fcw @1;
      steerRequired @2;
      brakePressed @3;
      wrongGear @4;
      seatbeltUnbuckled @5;
      speedTooHigh @6;
      ldw @7;
    }

    enum AudibleAlert {
      none @0;

      engage @1;
      disengage @2;
      refuse @3;

      warningSoft @4;
      warningImmediate @5;

      prompt @6;
      promptRepeat @7;
      promptDistracted @8;

      slowingDownSpeed @9;
    }
  }

  gasDEPRECATED @1 :Float32;
  brakeDEPRECATED @2 :Float32;
  steeringTorqueDEPRECATED @3 :Float32;
  activeDEPRECATED @7 :Bool;
  rollDEPRECATED @8 :Float32;
  pitchDEPRECATED @9 :Float32;
}

# ****** car param ******

struct CarParams {
  carName @0 :Text;
  carFingerprint @1 :Text;
  fuzzyFingerprint @55 :Bool;

  notCar @66 :Bool;

  enableGasInterceptor @2 :Bool;
  pcmCruise @3 :Bool;
  enableDsu @5 :Bool;
  enableApgs @6 :Bool;
  enableBsm @56 :Bool;
  flags @64 :UInt32;

  minEnableSpeed @7 :Float32;
  minSteerSpeed @8 :Float32;
  maxSteeringAngleDeg @54 :Float32;
  safetyConfigs @62 :List(SafetyConfig);
  alternativeExperience @65 :Int16;
  maxLateralAccel @68 :Float32;

  steerMaxBPDEPRECATED @11 :List(Float32);
  steerMaxVDEPRECATED @12 :List(Float32);
  gasMaxBPDEPRECATED @13 :List(Float32);
  gasMaxVDEPRECATED @14 :List(Float32);
  brakeMaxBPDEPRECATED @15 :List(Float32);
  brakeMaxVDEPRECATED @16 :List(Float32);

  mass @17 :Float32;
  wheelbase @18 :Float32;
  centerToFront @19 :Float32;
  steerRatio @20 :Float32;
  steerRatioRear @21 :Float32;

  rotationalInertia @22 :Float32;
  tireStiffnessFront @23 :Float32;
  tireStiffnessRear @24 :Float32;

  longitudinalTuning @25 :LongitudinalPIDTuning;
  lateralParams @48 :LateralParams;
  lateralTuning :union {
    pid @26 :LateralPIDTuning;
    indi @27 :LateralINDITuning;
    lqr @40 :LateralLQRTuning;
    torque @67 :LateralTorqueTuning;
  }

  steerLimitAlert @28 :Bool;
  steerLimitTimer @47 :Float32;

  vEgoStopping @29 :Float32;
  vEgoStarting @59 :Float32;
  directAccelControl @30 :Bool;
  stoppingControl @31 :Bool;
  steerControlType @34 :SteerControlType;
  radarOffCan @35 :Bool;
  stopAccel @60 :Float32;
  stoppingDecelRate @52 :Float32;
  startAccel @32 :Float32;
  startingState @78 :Bool;
  steerActuatorDelay @36 :Float32;
  longitudinalActuatorDelayUpperBound @58 :Float32;
  longitudinalActuatorDelayLowerBound @61 :Float32;
  openpilotLongitudinalControl @37 :Bool;
  carVin @38 :Text;
  dashcamOnly @41: Bool;
  transmissionType @43 :TransmissionType;
  carFw @44 :List(CarFw);

  radarTimeStep @45: Float32 = 0.05;
  fingerprintSource @49: FingerprintSource;
  networkLocation @50 :NetworkLocation;

  wheelSpeedFactor @63 :Float32;
  pfeiferjDesiredCurvatures @79 :Bool;

  struct SafetyConfig {
    safetyModel @0 :SafetyModel;
    safetyParam @1 :Int16;
  }

  mdpsBus @69: Int8;
  sasBus @70: Int8;
  sccBus @71: Int8;
  enableAutoHold @72 :Bool;
  hasScc13 @73 :Bool;
  hasScc14 @74 :Bool;
  hasEms @75 :Bool;
  hasLfaHda @76 :Bool;

  disableLateralLiveTuning @77 :Bool;

  struct LateralParams {
    torqueBP @0 :List(Int32);
    torqueV @1 :List(Int32);
  }

  struct LateralPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    kf @4 :Float32;
    kdBP @5 :List(Float32) = [0.];
    kdV @6 :List(Float32) = [0.];
    newKfTuned @7 :Bool;
  }

  struct LateralTorqueTuning {
    useSteeringAngle @0 :Bool;
    kp @1 :Float32;
    ki @2 :Float32;
    kf @3 :Float32;
    kd @4 :Float32;
    friction @5 :Float32;
    steeringAngleDeadzoneDeg @6 :Float32;
    latAccelFactor @7 :Float32;
    latAccelOffset @8 :Float32;
    nnModelName @9 :Text;
    nnModelFuzzyMatch @10 :Bool;
  }

  struct LongitudinalPIDTuning {
    kpBP @0 :List(Float32);
    kpV @1 :List(Float32);
    kiBP @2 :List(Float32);
    kiV @3 :List(Float32);
    kf @8 :Float32;
    deadzoneBP @4 :List(Float32);
    deadzoneV @5 :List(Float32);
    kdBP @6 :List(Float32) = [0.];
    kdV @7 :List(Float32) = [0.];
  }

  struct LateralINDITuning {
    outerLoopGainBP @4 :List(Float32);
    outerLoopGainV @5 :List(Float32);
    innerLoopGainBP @6 :List(Float32);
    innerLoopGainV @7 :List(Float32);
    timeConstantBP @8 :List(Float32);
    timeConstantV @9 :List(Float32);
    actuatorEffectivenessBP @10 :List(Float32);
    actuatorEffectivenessV @11 :List(Float32);

    outerLoopGainDEPRECATED @0 :Float32;
    innerLoopGainDEPRECATED @1 :Float32;
    timeConstantDEPRECATED @2 :Float32;
    actuatorEffectivenessDEPRECATED @3 :Float32;
  }

  struct LateralLQRTuning {
    scale @0 :Float32;
    ki @1 :Float32;
    dcGain @2 :Float32;

    a @3 :List(Float32);
    b @4 :List(Float32);
    c @5 :List(Float32);

    k @6 :List(Float32);
    l @7 :List(Float32);
  }

  enum SafetyModel {
    silent @0;
    hondaNidec @1;
    toyota @2;
    elm327 @3;
    gm @4;
    hondaBoschGiraffe @5;
    ford @6;
    cadillac @7;
    hyundai @8;
    chrysler @9;
    tesla @10;
    subaru @11;
    gmPassive @12;
    mazda @13;
    nissan @14;
    volkswagen @15;
    toyotaIpas @16;
    allOutput @17;
    gmAscm @18;
    noOutput @19;
    hondaBosch @20;
    volkswagenPq @21;
    subaruLegacy @22;
    hyundaiLegacy @23;
    hyundaiCommunity @24;
    stellantis @25;
    faw @26;
    body @27;
  }

  enum SteerControlType {
    torque @0;
    angle @1;
  }

  enum TransmissionType {
    unknown @0;
    automatic @1;
    manual @2;
    direct @3;
    cvt @4;
  }

  struct CarFw {
    ecu @0 :Ecu;
    fwVersion @1 :Data;
    address @2: UInt32;
    subAddress @3: UInt8;
  }

  enum Ecu {
    eps @0;
    esp @1;
    fwdRadar @2;
    fwdCamera @3;
    engine @4;
    unknown @5;
    transmission @8;
    srs @9;
    gateway @10;
    hud @11;
    combinationMeter @12;

    dsu @6;
    apgs @7;

    vsa @13;
    programmedFuelInjection @14;
    electricBrakeBooster @15;
    shiftByWire @16;

    debug @17;
  }

  enum FingerprintSource {
    can @0;
    fw @1;
    fixed @2;
  }

  enum NetworkLocation {
    fwdCamera @0;
    gateway @1;
  }

  enableCameraDEPRECATED @4 :Bool;
  steerRateCostDEPRECATED @33 :Float32;
  isPandaBlackDEPRECATED @39 :Bool;
  hasStockCameraDEPRECATED @57 :Bool;
  safetyParamDEPRECATED @10 :Int16;
  safetyModelDEPRECATED @9 :SafetyModel;
  safetyModelPassiveDEPRECATED @42 :SafetyModel = silent;
  minSpeedCanDEPRECATED @51 :Float32;
  communityFeatureDEPRECATED @46: Bool;
  startingAccelRateDEPRECATED @53 :Float32;
}
