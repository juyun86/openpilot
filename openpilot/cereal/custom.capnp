using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct ModularAssistiveDrivingSystem {
  state @0 :ModularAssistiveDrivingSystemState;
  enabled @1 :Bool;
  active @2 :Bool;
  available @3 :Bool;

  enum ModularAssistiveDrivingSystemState {
    disabled @0;
    paused @1;
    enabled @2;
    softDisabling @3;
    overriding @4;
  }
}

struct IntelligentCruiseButtonManagement {
  state @0 :IntelligentCruiseButtonManagementState;
  sendButton @1 :SendButtonState;
  vTarget @2 :Float32;

  enum IntelligentCruiseButtonManagementState {
    inactive @0;      # No button press or default state
    preActive @1;     # Pre-active state before transitioning to increasing or decreasing
    increasing @2;    # Increasing speed
    decreasing @3;    # Decreasing speed
    holding @4;       # Holding steady speed
  }

  enum SendButtonState {
    none @0;
    increase @1;
    decrease @2;
  }
}

# Same struct as Log.RadarState.LeadData
struct LeadData {
  dRel @0 :Float32;
  yRel @1 :Float32;
  vRel @2 :Float32;
  aRel @3 :Float32;
  vLead @4 :Float32;
  dPath @6 :Float32;
  vLat @7 :Float32;
  vLeadK @8 :Float32;
  aLeadK @9 :Float32;
  fcw @10 :Bool;
  status @11 :Bool;
  aLeadTau @12 :Float32;
  modelProb @13 :Float32;
  radar @14 :Bool;
  radarTrackId @15 :Int32 = -1;

  aLeadDEPRECATED @5 :Float32;
}

struct SelfdriveStateSP @0x81c2f05a394cf4af {
  mads @0 :ModularAssistiveDrivingSystem;
  intelligentCruiseButtonManagement @1 :IntelligentCruiseButtonManagement;
  buttonsPressed @2 :UInt16;
  buttonsReleaseToggle @3 :UInt16;

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

    # unused, these are reserved for upstream events so we don't collide
    reserved9 @9;
    reserved10 @10;
    reserved11 @11;
    reserved12 @12;
    reserved13 @13;
    reserved14 @14;
    reserved15 @15;
    reserved16 @16;
    reserved17 @17;
    reserved18 @18;
    reserved19 @19;
    reserved20 @20;
    reserved21 @21;
    reserved22 @22;
    reserved23 @23;
    reserved24 @24;
    reserved25 @25;
    reserved26 @26;
    reserved27 @27;
    reserved28 @28;
    reserved29 @29;
    reserved30 @30;

    promptSingleLow @31;
    promptSingleHigh @32;
  }
}

struct ModelManagerSP @0xaedffd8f31e7b55d {
  activeBundle @0 :ModelBundle;
  selectedBundle @1 :ModelBundle;
  availableBundles @2 :List(ModelBundle);

  struct DownloadUri {
    uri @0 :Text;
    sha256 @1 :Text;
  }

  enum DownloadStatus {
    notDownloading @0;
    downloading @1;
    downloaded @2;
    cached @3;
    failed @4;
    verifying @5;
  }

  struct DownloadProgress {
    status @0 :DownloadStatus;
    progress @1 :Float32;
    eta @2 :UInt32;
  }

  struct Chunk {
    fileName @0 :Text;
    sha256 @1 :Text;
  }

  struct Artifact {
    fileName @0 :Text;
    downloadUri @1 :DownloadUri;
    downloadProgress @2 :DownloadProgress;
    chunks @3 :List(Chunk);
  }

  struct Model {
    type @0 :Type;
    artifact @1 :Artifact;  # Main artifact
    metadata @2 :Artifact;  # Metadata artifact

    enum Type {
      supercombo @0;
      navigation @1;
      vision @2;
      policy @3;
      offPolicy @4;
      onPolicy @5;
      chunked @6;
    }
  }

  enum Runner {
    snpe @0;
    tinygrad @1;
    stock @2;
  }

  struct Override {
    key @0 :Text;
    value @1 :Text;
  }

  struct ModelBundle {
    index @0 :UInt32;
    internalName @1 :Text;
    displayName @2 :Text;
    models @3 :List(Model);
    status @4 :DownloadStatus;
    generation @5 :UInt32;
    environment @6 :Text;
    runner @7 :Runner;
    is20hz @8 :Bool;
    ref @9 :Text;
    minimumSelectorVersion @10 :UInt32;
    overrides @11 :List(Override);
  }
}

struct LongitudinalPlanSP @0xf35cc4560bbf6ec2 {
  dec @0 :DynamicExperimentalControl;
  longitudinalPlanSource @1 :LongitudinalPlanSource;
  smartCruiseControl @2 :SmartCruiseControl;
  speedLimit @3 :SpeedLimit;
  vTarget @4 :Float32;
  aTarget @5 :Float32;
  events @6 :List(OnroadEventSP.Event);
  e2eAlerts @7 :E2eAlerts;
  accelController @8 :AccelController;
  teslaTrafficControl @9 :TeslaTrafficControlPlan;

  struct DynamicExperimentalControl {
    state @0 :DynamicExperimentalControlState;
    enabled @1 :Bool;
    active @2 :Bool;

    enum DynamicExperimentalControlState {
      acc @0;
      blended @1;
    }
  }

  struct SmartCruiseControl {
    vision @0 :Vision;
    map @1 :Map;

    struct Vision {
      state @0 :VisionState;
      vTarget @1 :Float32;
      aTarget @2 :Float32;
      currentLateralAccel @3 :Float32;
      maxPredictedLateralAccel @4 :Float32;
      enabled @5 :Bool;
      active @6 :Bool;
    }

    struct Map {
      state @0 :MapState;
      vTarget @1 :Float32;
      aTarget @2 :Float32;
      enabled @3 :Bool;
      active @4 :Bool;
    }

    enum VisionState {
      disabled @0; # System disabled or inactive.
      enabled @1; # No predicted substantial turn on vision range.
      entering @2; # A substantial turn is predicted ahead, adapting speed to turn comfort levels.
      turning @3; # Actively turning. Managing acceleration to provide a roll on turn feeling.
      leaving @4; # Road ahead straightens. Start to allow positive acceleration.
      overriding @5; # System overriding with manual control.
    }

    enum MapState {
      disabled @0; # System disabled or inactive.
      enabled @1; # No predicted substantial turn on map range.
      turning @2; # Actively turning. Managing acceleration to provide a roll on turn feeling.
      overriding @3; # System overriding with manual control.
    }
  }

  struct SpeedLimit {
    resolver @0 :Resolver;
    assist @1 :Assist;

    struct Resolver {
      speedLimit @0 :Float32;
      distToSpeedLimit @1 :Float32;
      source @2 :Source;
      speedLimitOffset @3 :Float32;
      speedLimitLast @4 :Float32;
      speedLimitFinal @5 :Float32;
      speedLimitFinalLast @6 :Float32;
      speedLimitValid @7 :Bool;
      speedLimitLastValid @8 :Bool;
    }

    struct Assist {
      state @0 :AssistState;
      enabled @1 :Bool;
      active @2 :Bool;
      vTarget @3 :Float32;
      aTarget @4 :Float32;
    }

    enum Source {
      none @0;
      car @1;
      map @2;
    }

    enum AssistState {
      disabled @0;
      inactive @1; # No speed limit set or not enabled by parameter.
      preActive @2;
      pending @3; # Awaiting new speed limit.
      adapting @4; # Reducing speed to match new speed limit.
      active @5; # Cruising at speed limit.
    }
  }

  enum LongitudinalPlanSource {
    cruise @0;
    sccVision @1;
    sccMap @2;
    speedLimitAssist @3;
  }

  struct E2eAlerts {
    greenLightAlert @0 :Bool;
    leadDepartAlert @1 :Bool;
  }

  struct AccelController {
    enabled @0 :Bool;
    active @1 :Bool;
    shadowOnlyDEPRECATED @2 :Bool;
    profile @3 :Profile;
    state @4 :State;

    enum Profile {
      eco @0;
      normal @1;
      sport @2;
    }

    enum State {
      inactive @0;
      free @1;
      restrict @2;
      hold @3;
      release @4;
      stopHold @5;
    }
  }
}

struct OnroadEventSP @0xda96579883444c35 {
  events @0 :List(Event);

  struct Event {
    name @0 :EventName;

    # event types
    enable @1 :Bool;
    noEntry @2 :Bool;
    warning @3 :Bool;   # alerts presented only when  enabled or soft disabling
    userDisable @4 :Bool;
    softDisable @5 :Bool;
    immediateDisable @6 :Bool;
    preEnable @7 :Bool;
    permanent @8 :Bool; # alerts presented regardless of openpilot state
    overrideLateral @10 :Bool;
    overrideLongitudinal @9 :Bool;
  }

  enum EventName {
    lkasEnable @0;
    lkasDisable @1;
    manualSteeringRequired @2;
    manualLongitudinalRequired @3;
    silentLkasEnable @4;
    silentLkasDisable @5;
    silentBrakeHold @6;
    silentWrongGear @7;
    silentReverseGear @8;
    silentDoorOpen @9;
    silentSeatbeltNotLatched @10;
    silentParkBrake @11;
    controlsMismatchLateral @12;
    hyundaiRadarTracksConfirmed @13;
    experimentalModeSwitched @14;
    wrongCarModeAlertOnly @15;
    pedalPressedAlertOnly @16;
    laneTurnLeft @17;
    laneTurnRight @18;
    speedLimitPreActive @19;
    speedLimitActive @20;
    speedLimitChanged @21;
    speedLimitPending @22;
    e2eChime @23;
    laneChangeRoadEdge @24;
  }
}

struct CarParamsSP @0x80ae746ee2596b11 {
  flags @0 :UInt32;        # flags for car specific quirks in sunnypilot
  safetyParam @1 : Int16;  # flags for sunnypilot's custom safety flags
  pcmCruiseSpeed @3 :Bool;
  intelligentCruiseButtonManagementAvailable @4 :Bool;
  enableGasInterceptor @5 :Bool;

  neuralNetworkLateralControl @2 :NeuralNetworkLateralControl;

  struct NeuralNetworkLateralControl {
    model @0 :Model;
    fuzzyFingerprint @1 :Bool;

    struct Model {
      path @0 :Text;
      name @1 :Text;
    }
  }
}

struct CarControlSP @0xa5cd762cd951a455 {
  mads @0 :ModularAssistiveDrivingSystem;
  params @1 :List(Param);
  leadOne @2 :LeadData;
  leadTwo @3 :LeadData;
  intelligentCruiseButtonManagement @4 :IntelligentCruiseButtonManagement;

  struct Param {
    key @0 :Text;
    type @2 :ParamType;
    value @3 :Data;

    valueDEPRECATED @1 :Text; # The data type change may cause issues with backwards compatibility.
  }

  enum ParamType {
    string @0;
    bool @1;
    int @2;
    float @3;
    time @4;
    json @5;
    bytes @6;
  }
}

struct BackupManagerSP @0xf98d843bfd7004a3 {
  backupStatus @0 :Status;
  restoreStatus @1 :Status;
  backupProgress @2 :Float32;
  restoreProgress @3 :Float32;
  lastError @4 :Text;
  currentBackup @5 :BackupInfo;
  backupHistory @6 :List(BackupInfo);

  enum Status {
    idle @0;
    inProgress @1;
    completed @2;
    failed @3;
  }

  struct Version {
    major @0 :UInt16;
    minor @1 :UInt16;
    patch @2 :UInt16;
    build @3 :UInt16;
    branch @4 :Text;
  }

  struct MetadataEntry {
    key @0 :Text;
    value @1 :Text;
    tags @2 :List(Text);
  }

  struct BackupInfo {
    deviceId @0 :Text;
    version @1 :UInt32;
    config @2 :Text;
    isEncrypted @3 :Bool;
    createdAt @4 :Text;  # ISO timestamp
    updatedAt @5 :Text;  # ISO timestamp
    sunnypilotVersion @6 :Version;
    backupMetadata @7 :List(MetadataEntry);
  }
}

struct CarStateSP @0xb86e6369214c01c8 {
  speedLimit @0 :Float32;
  flags @1 :UInt32;  # Optional car-module runtime flags (Tesla split-control ownership).
  teslaRoadContext @2 :TeslaRoadContext;
  teslaTrafficControl @3 :TeslaTrafficControl;
}

struct TeslaRoadContext {
  available @0 :Bool;
  trafficLightColor @1 :UInt8;
  stopLineDistance @2 :Float32;
}

struct TeslaTrafficControl {
  available @0 :Bool;
  validForControl @1 :Bool;
  sourceBus @2 :UInt8;
  dlc @3 :UInt8;
  featureState @4 :UInt8;
  stateMachine @5 :UInt8;
  controlSource @6 :UInt8;
  controlType @7 :UInt8;
  distance @8 :Float32;
  lightState @9 :UInt8;
  continuationReason @10 :UInt8;
  confirmationType @11 :UInt8;
  warningSuppressionReason @12 :UInt8;
  unavailableReason @13 :UInt8;
  visionLight @14 :Bool;
  visionSign @15 :Bool;
  visionRoadMarking @16 :Bool;
  visionLine @17 :Bool;
  frameMonoTime @18 :UInt64;
  quality @19 :UInt8;
}

struct TeslaTrafficControlPlan {
  mode @0 :UInt8;
  phase @1 :UInt8;
  active @2 :Bool;
  shadow @3 :Bool;
  applied @4 :Bool;
  shouldStop @5 :Bool;
  remainingDistance @6 :Float32;
  stopReference @7 :Float32;
  lightState @8 :UInt8;
  sourceBus @9 :UInt8;
  quality @10 :UInt8;
  constraintAccel @11 :Float32;
  action @12 :UInt8;
  baseATarget @13 :Float32;
  finalATarget @14 :Float32;
  startRequested @15 :Bool;
  startApplied @16 :Bool;
  startBlockReason @17 :UInt8;
  eventId @18 :UInt32;
  terminalCatchActive @19 :Bool;
  rawDistance @20 :Float32;
  stopSessionId @21 :UInt32;
  directionUnknown @22 :Bool;
  driverOverrideActive @23 :Bool;
  canRemaining @24 :Float32;
  stationInnovation @25 :Float32;
  stopControlAllowed @26 :Bool;
  rawObservationFresh @27 :Bool;
  rawObservationAgeMs @28 :Float32;
  stopDirectionUnknown @29 :Bool;
  stopSafetyAllowed @30 :Bool;  # All STOP gates except raw CAN freshness.
}

struct LiveMapDataSP @0xf416ec09499d9d19 {
  speedLimitValid @0 :Bool;
  speedLimit @1 :Float32;
  speedLimitAheadValid @2 :Bool;
  speedLimitAhead @3 :Float32;
  speedLimitAheadDistance @4 :Float32;
  roadName @5 :Text;
}

struct ModelDataV2SP @0xa1680744031fdb2d {
  laneTurnDirection @0 :TurnDirection;
  leftLaneChangeEdgeBlock @1 :Bool;
  rightLaneChangeEdgeBlock @2 :Bool;

  enum TurnDirection {
    none @0;
    turnLeft @1;
    turnRight @2;
  }
}

struct TrafficRadarState @0xcb9fd56c7057593a {
  # Legacy-named independent traffic-control target. It is never a physical
  # vehicle and must not be forwarded to radarState, modelV2, FCW, car state,
  # or vehicle CAN.
  targetPresent @0 :Bool;
  oemTargetDistance @1 :Float32;
  targetRelativeVelocity @2 :Float32;
  targetRelativeAcceleration @3 :Float32;
  distanceToStopPoint @4 :Float32;
  phase @5 :UInt8;
  lightState @6 :UInt8;
  sourceBus @7 :UInt8;
  quality @8 :UInt8;
  confidence @9 :Float32;
  eventId @10 :UInt32;
  publishMonoTime @11 :UInt64;
  controlAllowed @12 :Bool;
  suppressedByPhysicalLead @13 :Bool;  # Deprecated; traffic control does not consume radarState.
  shouldStop @14 :Bool;
  plannerStartRequested @15 :Bool;
  mode @16 :UInt8;
  rawGreenSeen @17 :Bool;
  releaseEligible @18 :Bool;
  eventContinuous @19 :Bool;
  eventTransitionReason @20 :UInt8;
  eventTransitionSeq @21 :UInt32;
  rawDistance @22 :Float32;
  observationAgeMs @23 :Float32;
  stopSessionId @24 :UInt32;
  directionUnknown @25 :Bool;
  driverOverrideActive @26 :Bool;
  canRemaining @27 :Float32;
  stationInnovation @28 :Float32;
  stopControlAllowed @29 :Bool;
  rawObservationFresh @30 :Bool;
  stopDirectionUnknown @31 :Bool;
  stopSafetyAllowed @32 :Bool;  # All STOP gates except raw CAN freshness.
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct ARS408StateSP @0x9ccdc8676701b412 {
  sequence @0 :UInt32;
  measurementMonoTime @1 :UInt64;
  measurementCounter @2 :UInt16;
  cycleStatus @3 :CycleStatus;
  motionInputValid @4 :Bool;
  generalComplete @5 :Bool;
  qualityComplete @6 :Bool;
  extendedComplete @7 :Bool;
  rawCount @8 :UInt16;
  targets @9 :List(CompactTarget);
  # Deprecated wire-compatibility slots. Full diagnostics moved to the
  # independent ars408DiagnosticsSP service; producers must keep these empty.
  diagnosticTargets @10 :List(DiagnosticTarget);
  stableCount @11 :UInt16;
  uncertainCount @12 :UInt16;
  clutterCount @13 :UInt16;
  expiredCount @14 :UInt16;
  targetCount @15 :UInt16;
  diagnosticCount @16 :UInt16;
  targetsTruncated @17 :Bool;
  diagnosticsTruncated @18 :Bool;
  diagnosticsUpdated @19 :Bool;
  diagnosticSequence @20 :UInt32;
  schemaVersion @21 :UInt16;
  health @22 :Health;
  hasData @23 :Bool;
  sourceFresh @24 :Bool;
  sourceAgeMs @25 :UInt32;
  radarStateReady @26 :Bool;
  radarStateFresh @27 :Bool;
  parserValid @28 :Bool;
  producerFault @29 :Bool;
  producerFaultCount @30 :UInt32;
  errors @31 :Errors;
  counterGap @32 :UInt16;
  droppedCycleCount @33 :UInt32;
  acceptedCount @34 :UInt16;
  forwardPresenceState @35 :ForwardPresenceState;
  lateralTransformValid @36 :Bool;
  lastComponentMonoTime @37 :UInt64;
  assemblyCloseMonoTime @38 :UInt64;
  interferenceCount @39 :UInt32;
  interferenceActive @40 :Bool;
  cadenceValid @41 :Bool;
  expectedCounterStep @42 :UInt16;
  configObserved @43 :Bool;
  configValid @44 :Bool;
  counterAnomalyCount @45 :UInt32;
  cadenceVerified @46 :Bool;
  cadenceInferred @47 :Bool;
  stepSource @48 :CadenceStepSource;
  # Failures observed anywhere within this producer update, even if the final
  # sampled RadarState recovered before publication.
  transientFailureBits @49 :UInt32;
  # Non-zero process-instance/hard-reset identity. Zero is unknown. sequence
  # and logicalId are meaningful only together with this epoch. Any future
  # consumer must independently enforce service liveness, local TTL, monotonic
  # Event/source time, and discard state on epoch change; a teardown barrier is
  # best-effort and cannot make a last valid socket message safe indefinitely.
  producerEpoch @50 :UInt64;

  enum CycleStatus {
    invalid @0;
    exact @1;
    partial @2;
    duplicate @3;
    unavailable @4;
  }

  enum Lifecycle {
    unavailable @0;
    tentative @1;
    confirmedFresh @2;
    coasting @3;
    clutterSuspect @4;
    expired @5;
  }

  enum SemanticGroup {
    unknownObstacle @0;
    vehicle @1;
    pedestrian @2;
    rider @3;
  }

  enum Health {
    unavailable @0;
    healthy @1;
    degraded @2;
    stale @3;
    producerFault @4;
  }

  # Presence-only fail-safe state. It never grants permission to proceed and
  # must not be interpreted as lane use or a permissive road-edge/BSM signal.
  enum ForwardPresenceState {
    unknown @0;
    present @1;
  }

  enum CadenceStepSource {
    unknown @0;
    external @1;
    inferred @2;
    deviceCapture @3;
  }

  enum DiagnosticKind {
    unknown @0;
    uncertain @1;
    clutter @2;
    expired @3;
    stableAudit @4;
  }

  struct Errors {
    canError @0 :Bool;
    radarFault @1 :Bool;
    radarUnavailableTemporary @2 :Bool;
    wrongConfig @3 :Bool;
  }

  struct CompactTarget {
    # dRel/yRel use metres (m); vRel uses m/s; lastMeasuredAgeMs uses ms.
    logicalId @0 :UInt64;
    rawId @1 :UInt16;
    lifecycle @2 :Lifecycle;
    semanticGroup @3 :SemanticGroup;
    freshMeasured @4 :Bool;
    possibleVru @5 :Bool;
    dRel @6 :Float32;
    yRel @7 :Float32;
    vRel @8 :Float32;
    lastMeasuredAgeMs @9 :Float32;
    ageCycles @10 :UInt32;
    existenceProbabilityCode @11 :UInt8;
    measurementState @12 :UInt8;
    dynamicProperty @13 :UInt8;
    rawClass @14 :UInt8;
    stableClass @15 :UInt8;
    reasonBits @16 :UInt32;
  }

  struct DiagnosticTarget {
    # Diagnostic units: rawDRel/rawYRel/length/width use metres (m);
    # rawVRel/rawYvRel/yvRel use m/s; aRelLong/aRelLat use m/s²;
    # orientation uses degrees; rcs uses dBm² (DBC spelling: dBm2).
    # All lateral fields yRel/rawYRel/yvRel/rawYvRel/aRelLat/orientation have
    # passed through the current sensor-to-openpilot sign transform, which
    # remains unverified on the installed vehicle.
    # observedProducerEpoch, observedSequence, and observedMonoTime identify
    # the selected source observation, not the diagnostics publication header.
    kind @0 :DiagnosticKind;
    target @1 :CompactTarget;
    extendedFresh @2 :Bool;
    hitCount @3 :UInt32;
    missCount @4 :UInt32;
    rawDRel @5 :Float32;
    rawYRel @6 :Float32;
    rawVRel @7 :Float32;
    rawYvRel @8 :Float32;
    aRelLong @9 :Float32;
    aRelLat @10 :Float32;
    orientation @11 :Float32;
    length @12 :Float32;
    width @13 :Float32;
    rcs @14 :Float32;
    clutterScore @15 :Float32;
    rmsCodes @16 :List(UInt8);
    rmsUpperBounds @17 :List(Float32);
    rmsValidMask @18 :UInt8;
    yvRel @19 :Float32;
    qualityScore @20 :Float32;
    coordinateUnverified @21 :Bool;
    observedSequence @22 :UInt32;
    observedMonoTime @23 :UInt64;
    observedProducerEpoch @24 :UInt64;
  }
}

# Window audit contract: targets, fullCount, targetCount, and the four
# *FullCount fields form a bounded incremental audit collection accumulated
# since the previous physical ars408DiagnosticsSP event. Selection is
# deduplicated by (observedProducerEpoch, logicalId): possible-VRU/class-conflict records have highest
# safety priority, then expired, clutter, uncertain, and stableAudit; for the
# same (observedProducerEpoch, logicalId) and priority, the newer observation replaces the older one.
# rawCount and acceptedCount describe only the latest snapshot header; they are
# not window totals. A targetCount of zero is not a current target-absence claim
# and must never authorize free space, lane use, or motion.
struct ARS408DiagnosticsSP @0xcd96dafb67a082d0 {
  schemaVersion @0 :UInt16;
  sequence @1 :UInt32;
  measurementMonoTime @2 :UInt64;
  health @3 :ARS408StateSP.Health;
  hasData @4 :Bool;
  sourceFresh @5 :Bool;
  sourceAgeMs @6 :UInt32;
  coordinateUnverified @7 :Bool;
  fullCount @8 :UInt16;
  targetCount @9 :UInt16;
  truncated @10 :Bool;
  targets @11 :List(ARS408StateSP.DiagnosticTarget);
  lastComponentMonoTime @12 :UInt64;
  assemblyCloseMonoTime @13 :UInt64;
  parserValid @14 :Bool;
  errors @15 :ARS408StateSP.Errors;
  radarStateReady @16 :Bool;
  radarStateFresh @17 :Bool;
  configObserved @18 :Bool;
  configValid @19 :Bool;
  interferenceActive @20 :Bool;
  interferenceCount @21 :UInt32;
  motionInputValid @22 :Bool;
  lateralTransformValid @23 :Bool;
  producerFault @24 :Bool;
  producerFaultCount @25 :UInt32;
  cycleStatus @26 :ARS408StateSP.CycleStatus;
  measurementCounter @27 :UInt16;
  counterGap @28 :UInt16;
  droppedCycleCount @29 :UInt32;
  counterAnomalyCount @30 :UInt32;
  cadenceValid @31 :Bool;
  cadenceVerified @32 :Bool;
  cadenceInferred @33 :Bool;
  expectedCounterStep @34 :UInt16;
  stepSource @35 :ARS408StateSP.CadenceStepSource;
  generalComplete @36 :Bool;
  qualityComplete @37 :Bool;
  extendedComplete @38 :Bool;
  rawCount @39 :UInt16;
  acceptedCount @40 :UInt16;
  stableFullCount @41 :UInt16;
  uncertainFullCount @42 :UInt16;
  clutterFullCount @43 :UInt16;
  expiredFullCount @44 :UInt16;
  windowStartMonoTime @45 :UInt64;
  windowEndMonoTime @46 :UInt64;
  windowReasonBits @47 :UInt32;
  transitionCount @48 :UInt32;
  worstHealth @49 :ARS408StateSP.Health;
  sourceFailed @50 :Bool;
  radarFailed @51 :Bool;
  parserFailed @52 :Bool;
  motionFailed @53 :Bool;
  configFailed @54 :Bool;
  interferenceFailed @55 :Bool;
  cadenceFailed @56 :Bool;
  counterFailed @57 :Bool;
  producerFailed @58 :Bool;
  transientFailureBits @59 :UInt32;
  # Same identity contract as ARS408StateSP.producerEpoch.
  producerEpoch @60 :UInt64;
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
