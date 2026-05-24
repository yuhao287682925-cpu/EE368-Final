
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ErrorCodes = require('./ErrorCodes.js');
let StepResponse = require('./StepResponse.js');
let PositionCommand = require('./PositionCommand.js');
let ControlLoop = require('./ControlLoop.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let TorqueOffset = require('./TorqueOffset.js');
let RampResponse = require('./RampResponse.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let Servoing = require('./Servoing.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let LoopSelection = require('./LoopSelection.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let CommandMode = require('./CommandMode.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let AxisOffsets = require('./AxisOffsets.js');
let AxisPosition = require('./AxisPosition.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let CommandFlags = require('./CommandFlags.js');
let WrenchCommand = require('./WrenchCommand.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let TwistCommand = require('./TwistCommand.js');
let OperatingMode = require('./OperatingMode.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let UserProfileList = require('./UserProfileList.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let SequenceInformation = require('./SequenceInformation.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let JointSpeed = require('./JointSpeed.js');
let UserProfile = require('./UserProfile.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let BridgeList = require('./BridgeList.js');
let JointAngles = require('./JointAngles.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let Mapping = require('./Mapping.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let NavigationDirection = require('./NavigationDirection.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let UserNotificationList = require('./UserNotificationList.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let Admittance = require('./Admittance.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let ControllerType = require('./ControllerType.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let MapGroupList = require('./MapGroupList.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let NetworkNotification = require('./NetworkNotification.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let GripperCommand = require('./GripperCommand.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let GpioAction = require('./GpioAction.js');
let Gripper = require('./Gripper.js');
let ActionHandle = require('./ActionHandle.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let Faults = require('./Faults.js');
let ControllerHandle = require('./ControllerHandle.js');
let MappingHandle = require('./MappingHandle.js');
let MapHandle = require('./MapHandle.js');
let ShapeType = require('./ShapeType.js');
let BridgeConfig = require('./BridgeConfig.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let ZoneShape = require('./ZoneShape.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let SystemTime = require('./SystemTime.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let BridgeType = require('./BridgeType.js');
let ControllerElementState = require('./ControllerElementState.js');
let MappingList = require('./MappingList.js');
let UserNotification = require('./UserNotification.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let GripperRequest = require('./GripperRequest.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let SnapshotType = require('./SnapshotType.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let WifiInformationList = require('./WifiInformationList.js');
let Query = require('./Query.js');
let Action = require('./Action.js');
let TransformationRow = require('./TransformationRow.js');
let BridgeStatus = require('./BridgeStatus.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let SequenceTasks = require('./SequenceTasks.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let PasswordChange = require('./PasswordChange.js');
let MapGroup = require('./MapGroup.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let Pose = require('./Pose.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let BackupEvent = require('./BackupEvent.js');
let ActionList = require('./ActionList.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let MapEvent_events = require('./MapEvent_events.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let SignalQuality = require('./SignalQuality.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let RFConfiguration = require('./RFConfiguration.js');
let GpioBehavior = require('./GpioBehavior.js');
let RobotEvent = require('./RobotEvent.js');
let UserEvent = require('./UserEvent.js');
let Delay = require('./Delay.js');
let TwistLimitation = require('./TwistLimitation.js');
let Base_Stop = require('./Base_Stop.js');
let RequestedActionType = require('./RequestedActionType.js');
let MapEvent = require('./MapEvent.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let IKData = require('./IKData.js');
let SafetyEvent = require('./SafetyEvent.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let ControllerList = require('./ControllerList.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let ControllerEventType = require('./ControllerEventType.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let NetworkEvent = require('./NetworkEvent.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let Sequence = require('./Sequence.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let IPv4Information = require('./IPv4Information.js');
let NetworkType = require('./NetworkType.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let ActionNotification = require('./ActionNotification.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let Map = require('./Map.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let JointTorque = require('./JointTorque.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let Waypoint = require('./Waypoint.js');
let ChangeTwist = require('./ChangeTwist.js');
let SequenceHandle = require('./SequenceHandle.js');
let Snapshot = require('./Snapshot.js');
let SoundType = require('./SoundType.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let ControllerNotification = require('./ControllerNotification.js');
let ServoingMode = require('./ServoingMode.js');
let GripperMode = require('./GripperMode.js');
let ControllerState = require('./ControllerState.js');
let ActionType = require('./ActionType.js');
let ChangeWrench = require('./ChangeWrench.js');
let BridgeResult = require('./BridgeResult.js');
let WifiEnableState = require('./WifiEnableState.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let GpioEvent = require('./GpioEvent.js');
let LimitationType = require('./LimitationType.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let FactoryEvent = require('./FactoryEvent.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let SequenceTask = require('./SequenceTask.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let FullUserProfile = require('./FullUserProfile.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let GpioCommand = require('./GpioCommand.js');
let NetworkHandle = require('./NetworkHandle.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let MapElement = require('./MapElement.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let Ssid = require('./Ssid.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let Twist = require('./Twist.js');
let Base_Position = require('./Base_Position.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let WrenchMode = require('./WrenchMode.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let Timeout = require('./Timeout.js');
let ProtectionZone = require('./ProtectionZone.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let ControllerInputType = require('./ControllerInputType.js');
let WaypointList = require('./WaypointList.js');
let EmergencyStop = require('./EmergencyStop.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let UserList = require('./UserList.js');
let Finger = require('./Finger.js');
let ActionEvent = require('./ActionEvent.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let FactoryNotification = require('./FactoryNotification.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let Orientation = require('./Orientation.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let WifiInformation = require('./WifiInformation.js');
let JointTorques = require('./JointTorques.js');
let JointLimitation = require('./JointLimitation.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let SequenceList = require('./SequenceList.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let ControllerEvent = require('./ControllerEvent.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let LedState = require('./LedState.js');
let MapList = require('./MapList.js');
let Wrench = require('./Wrench.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let JointAngle = require('./JointAngle.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let Point = require('./Point.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let Connection = require('./Connection.js');
let ArmState = require('./ArmState.js');
let UARTStopBits = require('./UARTStopBits.js');
let UARTSpeed = require('./UARTSpeed.js');
let DeviceHandle = require('./DeviceHandle.js');
let Empty = require('./Empty.js');
let SafetyHandle = require('./SafetyHandle.js');
let Unit = require('./Unit.js');
let NotificationHandle = require('./NotificationHandle.js');
let UARTParity = require('./UARTParity.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let DeviceTypes = require('./DeviceTypes.js');
let CountryCode = require('./CountryCode.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let SafetyNotification = require('./SafetyNotification.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let Permission = require('./Permission.js');
let Timestamp = require('./Timestamp.js');
let NotificationType = require('./NotificationType.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let UARTWordLength = require('./UARTWordLength.js');
let NotificationOptions = require('./NotificationOptions.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let KinematicLimits = require('./KinematicLimits.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let GravityVector = require('./GravityVector.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let PayloadInformation = require('./PayloadInformation.js');
let LinearTwist = require('./LinearTwist.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let CartesianTransform = require('./CartesianTransform.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let AngularTwist = require('./AngularTwist.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let IPv4Settings = require('./IPv4Settings.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let RunMode = require('./RunMode.js');
let CalibrationResult = require('./CalibrationResult.js');
let CalibrationElement = require('./CalibrationElement.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let PartNumber = require('./PartNumber.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let DeviceType = require('./DeviceType.js');
let SafetyInformation = require('./SafetyInformation.js');
let ModelNumber = require('./ModelNumber.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let RunModes = require('./RunModes.js');
let SafetyStatus = require('./SafetyStatus.js');
let Calibration = require('./Calibration.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let CalibrationItem = require('./CalibrationItem.js');
let SafetyEnable = require('./SafetyEnable.js');
let MACAddress = require('./MACAddress.js');
let SerialNumber = require('./SerialNumber.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let RebootRqst = require('./RebootRqst.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let DeviceHandles = require('./DeviceHandles.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let MotorFeedback = require('./MotorFeedback.js');
let MotorCommand = require('./MotorCommand.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let GPIOMode = require('./GPIOMode.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let I2CData = require('./I2CData.js');
let I2CDevice = require('./I2CDevice.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let GPIOValue = require('./GPIOValue.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let GPIOPull = require('./GPIOPull.js');
let I2CMode = require('./I2CMode.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let GPIOState = require('./GPIOState.js');
let EthernetDevice = require('./EthernetDevice.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let UARTPortId = require('./UARTPortId.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let BaseType = require('./BaseType.js');
let EndEffectorType = require('./EndEffectorType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let ModelId = require('./ModelId.js');
let VisionModuleType = require('./VisionModuleType.js');
let ArmLaterality = require('./ArmLaterality.js');
let BrakeType = require('./BrakeType.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let WristType = require('./WristType.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let BitRate = require('./BitRate.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let Option = require('./Option.js');
let VisionEvent = require('./VisionEvent.js');
let SensorSettings = require('./SensorSettings.js');
let ManualFocus = require('./ManualFocus.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let OptionValue = require('./OptionValue.js');
let FocusPoint = require('./FocusPoint.js');
let Resolution = require('./Resolution.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let Sensor = require('./Sensor.js');
let VisionNotification = require('./VisionNotification.js');
let FocusAction = require('./FocusAction.js');
let OptionInformation = require('./OptionInformation.js');
let FrameRate = require('./FrameRate.js');
let TranslationVector = require('./TranslationVector.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  SubErrorCodes: SubErrorCodes,
  ErrorCodes: ErrorCodes,
  StepResponse: StepResponse,
  PositionCommand: PositionCommand,
  ControlLoop: ControlLoop,
  CustomDataSelection: CustomDataSelection,
  TorqueOffset: TorqueOffset,
  RampResponse: RampResponse,
  ControlLoopSelection: ControlLoopSelection,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  VectorDriveParameters: VectorDriveParameters,
  ControlLoopParameters: ControlLoopParameters,
  FrequencyResponse: FrequencyResponse,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  Servoing: Servoing,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  LoopSelection: LoopSelection,
  CustomDataIndex: CustomDataIndex,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  CommandModeInformation: CommandModeInformation,
  TorqueCalibration: TorqueCalibration,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  CommandMode: CommandMode,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  AxisOffsets: AxisOffsets,
  AxisPosition: AxisPosition,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  StatusFlags: StatusFlags,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  CommandFlags: CommandFlags,
  WrenchCommand: WrenchCommand,
  ControllerElementHandle: ControllerElementHandle,
  TwistCommand: TwistCommand,
  OperatingMode: OperatingMode,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  ConstrainedOrientation: ConstrainedOrientation,
  UserProfileList: UserProfileList,
  WifiEncryptionType: WifiEncryptionType,
  ArmStateNotification: ArmStateNotification,
  SequenceInformation: SequenceInformation,
  MappingInfoNotification: MappingInfoNotification,
  ActionExecutionState: ActionExecutionState,
  JointSpeed: JointSpeed,
  UserProfile: UserProfile,
  RobotEventNotificationList: RobotEventNotificationList,
  BridgeList: BridgeList,
  JointAngles: JointAngles,
  Base_ControlModeNotification: Base_ControlModeNotification,
  Mapping: Mapping,
  TrajectoryErrorElement: TrajectoryErrorElement,
  AngularWaypoint: AngularWaypoint,
  NavigationDirection: NavigationDirection,
  ChangeJointSpeeds: ChangeJointSpeeds,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  WifiSecurityType: WifiSecurityType,
  SequenceTaskHandle: SequenceTaskHandle,
  SafetyNotificationList: SafetyNotificationList,
  OperatingModeNotificationList: OperatingModeNotificationList,
  UserNotificationList: UserNotificationList,
  WaypointValidationReport: WaypointValidationReport,
  Admittance: Admittance,
  FirmwareBundleVersions: FirmwareBundleVersions,
  ControllerType: ControllerType,
  CartesianSpeed: CartesianSpeed,
  MapGroupList: MapGroupList,
  SequenceTasksRange: SequenceTasksRange,
  NetworkNotification: NetworkNotification,
  RobotEventNotification: RobotEventNotification,
  GripperCommand: GripperCommand,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  GpioAction: GpioAction,
  Gripper: Gripper,
  ActionHandle: ActionHandle,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  AdmittanceMode: AdmittanceMode,
  ControllerConfigurationList: ControllerConfigurationList,
  Faults: Faults,
  ControllerHandle: ControllerHandle,
  MappingHandle: MappingHandle,
  MapHandle: MapHandle,
  ShapeType: ShapeType,
  BridgeConfig: BridgeConfig,
  WifiConfiguration: WifiConfiguration,
  ZoneShape: ZoneShape,
  ActivateMapHandle: ActivateMapHandle,
  SystemTime: SystemTime,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  ControllerBehavior: ControllerBehavior,
  BridgeType: BridgeType,
  ControllerElementState: ControllerElementState,
  MappingList: MappingList,
  UserNotification: UserNotification,
  ControllerNotificationList: ControllerNotificationList,
  GripperRequest: GripperRequest,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  WrenchLimitation: WrenchLimitation,
  SequenceTasksPair: SequenceTasksPair,
  BridgePortConfig: BridgePortConfig,
  NetworkNotificationList: NetworkNotificationList,
  SnapshotType: SnapshotType,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  WifiInformationList: WifiInformationList,
  Query: Query,
  Action: Action,
  TransformationRow: TransformationRow,
  BridgeStatus: BridgeStatus,
  CartesianLimitationList: CartesianLimitationList,
  SequenceTasks: SequenceTasks,
  Base_JointSpeeds: Base_JointSpeeds,
  PasswordChange: PasswordChange,
  MapGroup: MapGroup,
  ConstrainedPosition: ConstrainedPosition,
  ConstrainedJointAngle: ConstrainedJointAngle,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  ServoingModeNotificationList: ServoingModeNotificationList,
  ProtectionZoneInformation: ProtectionZoneInformation,
  Pose: Pose,
  GpioConfigurationList: GpioConfigurationList,
  BackupEvent: BackupEvent,
  ActionList: ActionList,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  Base_ServiceVersion: Base_ServiceVersion,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  MapEvent_events: MapEvent_events,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  ProtectionZoneHandle: ProtectionZoneHandle,
  SignalQuality: SignalQuality,
  Base_CapSenseMode: Base_CapSenseMode,
  BridgeIdentifier: BridgeIdentifier,
  RFConfiguration: RFConfiguration,
  GpioBehavior: GpioBehavior,
  RobotEvent: RobotEvent,
  UserEvent: UserEvent,
  Delay: Delay,
  TwistLimitation: TwistLimitation,
  Base_Stop: Base_Stop,
  RequestedActionType: RequestedActionType,
  MapEvent: MapEvent,
  CartesianLimitation: CartesianLimitation,
  IKData: IKData,
  SafetyEvent: SafetyEvent,
  CartesianWaypoint: CartesianWaypoint,
  BluetoothEnableState: BluetoothEnableState,
  ControllerList: ControllerList,
  MappingInfoNotificationList: MappingInfoNotificationList,
  FullIPv4Configuration: FullIPv4Configuration,
  ActionNotificationList: ActionNotificationList,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  ControllerElementEventType: ControllerElementEventType,
  TrajectoryInfoType: TrajectoryInfoType,
  ControllerEventType: ControllerEventType,
  ProtectionZoneList: ProtectionZoneList,
  NetworkEvent: NetworkEvent,
  ControllerNotification_state: ControllerNotification_state,
  Gen3GpioPinId: Gen3GpioPinId,
  Sequence: Sequence,
  AppendActionInformation: AppendActionInformation,
  IPv4Information: IPv4Information,
  NetworkType: NetworkType,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  WifiConfigurationList: WifiConfigurationList,
  ServoingModeInformation: ServoingModeInformation,
  ActionNotification: ActionNotification,
  OperatingModeInformation: OperatingModeInformation,
  Map: Map,
  Base_ControlModeInformation: Base_ControlModeInformation,
  Base_GpioConfiguration: Base_GpioConfiguration,
  JointTorque: JointTorque,
  TrajectoryErrorReport: TrajectoryErrorReport,
  Waypoint: Waypoint,
  ChangeTwist: ChangeTwist,
  SequenceHandle: SequenceHandle,
  Snapshot: Snapshot,
  SoundType: SoundType,
  ControlModeNotificationList: ControlModeNotificationList,
  SwitchControlMapping: SwitchControlMapping,
  JointNavigationDirection: JointNavigationDirection,
  ControllerNotification: ControllerNotification,
  ServoingMode: ServoingMode,
  GripperMode: GripperMode,
  ControllerState: ControllerState,
  ActionType: ActionType,
  ChangeWrench: ChangeWrench,
  BridgeResult: BridgeResult,
  WifiEnableState: WifiEnableState,
  Base_ControlMode: Base_ControlMode,
  GpioEvent: GpioEvent,
  LimitationType: LimitationType,
  MapGroupHandle: MapGroupHandle,
  FactoryEvent: FactoryEvent,
  ServoingModeNotification: ServoingModeNotification,
  SequenceTask: SequenceTask,
  ActuatorInformation: ActuatorInformation,
  OperatingModeNotification: OperatingModeNotification,
  Base_CapSenseConfig: Base_CapSenseConfig,
  ControllerConfigurationMode: ControllerConfigurationMode,
  TrajectoryInfo: TrajectoryInfo,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  FullUserProfile: FullUserProfile,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  GpioCommand: GpioCommand,
  NetworkHandle: NetworkHandle,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  MapElement: MapElement,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  Ssid: Ssid,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  Twist: Twist,
  Base_Position: Base_Position,
  Base_RotationMatrix: Base_RotationMatrix,
  WrenchMode: WrenchMode,
  SequenceInfoNotification: SequenceInfoNotification,
  Timeout: Timeout,
  ProtectionZone: ProtectionZone,
  IPv4Configuration: IPv4Configuration,
  ArmStateInformation: ArmStateInformation,
  ControllerInputType: ControllerInputType,
  WaypointList: WaypointList,
  EmergencyStop: EmergencyStop,
  ProtectionZoneNotification: ProtectionZoneNotification,
  JointsLimitationsList: JointsLimitationsList,
  UserList: UserList,
  Finger: Finger,
  ActionEvent: ActionEvent,
  GpioPinConfiguration: GpioPinConfiguration,
  FactoryNotification: FactoryNotification,
  TrajectoryErrorType: TrajectoryErrorType,
  Orientation: Orientation,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  WifiInformation: WifiInformation,
  JointTorques: JointTorques,
  JointLimitation: JointLimitation,
  FirmwareComponentVersion: FirmwareComponentVersion,
  SequenceList: SequenceList,
  ConstrainedJointAngles: ConstrainedJointAngles,
  ControllerConfiguration: ControllerConfiguration,
  ControllerEvent: ControllerEvent,
  ConstrainedPose: ConstrainedPose,
  LedState: LedState,
  MapList: MapList,
  Wrench: Wrench,
  ProtectionZoneEvent: ProtectionZoneEvent,
  JointAngle: JointAngle,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  Action_action_parameters: Action_action_parameters,
  Point: Point,
  TransformationMatrix: TransformationMatrix,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  ActuatorCustomData: ActuatorCustomData,
  BaseFeedback: BaseFeedback,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  ActuatorCommand: ActuatorCommand,
  Connection: Connection,
  ArmState: ArmState,
  UARTStopBits: UARTStopBits,
  UARTSpeed: UARTSpeed,
  DeviceHandle: DeviceHandle,
  Empty: Empty,
  SafetyHandle: SafetyHandle,
  Unit: Unit,
  NotificationHandle: NotificationHandle,
  UARTParity: UARTParity,
  SafetyStatusValue: SafetyStatusValue,
  DeviceTypes: DeviceTypes,
  CountryCode: CountryCode,
  UARTDeviceIdentification: UARTDeviceIdentification,
  SafetyNotification: SafetyNotification,
  CountryCodeIdentifier: CountryCodeIdentifier,
  Permission: Permission,
  Timestamp: Timestamp,
  NotificationType: NotificationType,
  UARTConfiguration: UARTConfiguration,
  UserProfileHandle: UserProfileHandle,
  CartesianReferenceFrame: CartesianReferenceFrame,
  UARTWordLength: UARTWordLength,
  NotificationOptions: NotificationOptions,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  KinematicLimits: KinematicLimits,
  KinematicLimitsList: KinematicLimitsList,
  GravityVector: GravityVector,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  PayloadInformation: PayloadInformation,
  LinearTwist: LinearTwist,
  ToolConfiguration: ToolConfiguration,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  ControlConfigurationEvent: ControlConfigurationEvent,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  ControlConfig_Position: ControlConfig_Position,
  DesiredSpeeds: DesiredSpeeds,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  CartesianTransform: CartesianTransform,
  ControlConfigurationNotification: ControlConfigurationNotification,
  AngularTwist: AngularTwist,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  IPv4Settings: IPv4Settings,
  CalibrationParameter_value: CalibrationParameter_value,
  SafetyThreshold: SafetyThreshold,
  SafetyInformationList: SafetyInformationList,
  FirmwareVersion: FirmwareVersion,
  SafetyConfiguration: SafetyConfiguration,
  SafetyConfigurationList: SafetyConfigurationList,
  RunMode: RunMode,
  CalibrationResult: CalibrationResult,
  CalibrationElement: CalibrationElement,
  BootloaderVersion: BootloaderVersion,
  PartNumber: PartNumber,
  CalibrationParameter: CalibrationParameter,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  DeviceType: DeviceType,
  SafetyInformation: SafetyInformation,
  ModelNumber: ModelNumber,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  RunModes: RunModes,
  SafetyStatus: SafetyStatus,
  Calibration: Calibration,
  CalibrationStatus: CalibrationStatus,
  CalibrationItem: CalibrationItem,
  SafetyEnable: SafetyEnable,
  MACAddress: MACAddress,
  SerialNumber: SerialNumber,
  CapSenseRegister: CapSenseRegister,
  RebootRqst: RebootRqst,
  PartNumberRevision: PartNumberRevision,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  DeviceHandles: DeviceHandles,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  MotorFeedback: MotorFeedback,
  MotorCommand: MotorCommand,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  I2CDeviceAddressing: I2CDeviceAddressing,
  GPIOMode: GPIOMode,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  I2CData: I2CData,
  I2CDevice: I2CDevice,
  I2CConfiguration: I2CConfiguration,
  I2CWriteParameter: I2CWriteParameter,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  GPIOValue: GPIOValue,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  GPIOPull: GPIOPull,
  I2CMode: I2CMode,
  I2CDeviceIdentification: I2CDeviceIdentification,
  GPIOState: GPIOState,
  EthernetDevice: EthernetDevice,
  GPIOIdentifier: GPIOIdentifier,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  UARTPortId: UARTPortId,
  GPIOIdentification: GPIOIdentification,
  I2CReadParameter: I2CReadParameter,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  EthernetConfiguration: EthernetConfiguration,
  EthernetSpeed: EthernetSpeed,
  EthernetDuplex: EthernetDuplex,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  BaseType: BaseType,
  EndEffectorType: EndEffectorType,
  InterfaceModuleType: InterfaceModuleType,
  ModelId: ModelId,
  VisionModuleType: VisionModuleType,
  ArmLaterality: ArmLaterality,
  BrakeType: BrakeType,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  WristType: WristType,
  SensorIdentifier: SensorIdentifier,
  BitRate: BitRate,
  SensorFocusAction: SensorFocusAction,
  IntrinsicParameters: IntrinsicParameters,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  OptionIdentifier: OptionIdentifier,
  DistortionCoefficients: DistortionCoefficients,
  Option: Option,
  VisionEvent: VisionEvent,
  SensorSettings: SensorSettings,
  ManualFocus: ManualFocus,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  OptionValue: OptionValue,
  FocusPoint: FocusPoint,
  Resolution: Resolution,
  ExtrinsicParameters: ExtrinsicParameters,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  Sensor: Sensor,
  VisionNotification: VisionNotification,
  FocusAction: FocusAction,
  OptionInformation: OptionInformation,
  FrameRate: FrameRate,
  TranslationVector: TranslationVector,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
};
