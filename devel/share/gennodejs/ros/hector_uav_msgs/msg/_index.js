
"use strict";

let RC = require('./RC.js');
let MotorCommand = require('./MotorCommand.js');
let HeightCommand = require('./HeightCommand.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let HeadingCommand = require('./HeadingCommand.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let RawRC = require('./RawRC.js');
let Compass = require('./Compass.js');
let RawMagnetic = require('./RawMagnetic.js');
let ThrustCommand = require('./ThrustCommand.js');
let RuddersCommand = require('./RuddersCommand.js');
let MotorStatus = require('./MotorStatus.js');
let Supply = require('./Supply.js');
let YawrateCommand = require('./YawrateCommand.js');
let ControllerState = require('./ControllerState.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let Altimeter = require('./Altimeter.js');
let RawImu = require('./RawImu.js');
let MotorPWM = require('./MotorPWM.js');
let ServoCommand = require('./ServoCommand.js');
let TakeoffFeedback = require('./TakeoffFeedback.js');
let LandingResult = require('./LandingResult.js');
let LandingAction = require('./LandingAction.js');
let PoseAction = require('./PoseAction.js');
let LandingFeedback = require('./LandingFeedback.js');
let TakeoffActionGoal = require('./TakeoffActionGoal.js');
let PoseActionFeedback = require('./PoseActionFeedback.js');
let TakeoffActionFeedback = require('./TakeoffActionFeedback.js');
let LandingActionResult = require('./LandingActionResult.js');
let TakeoffAction = require('./TakeoffAction.js');
let LandingActionFeedback = require('./LandingActionFeedback.js');
let LandingActionGoal = require('./LandingActionGoal.js');
let TakeoffResult = require('./TakeoffResult.js');
let TakeoffActionResult = require('./TakeoffActionResult.js');
let TakeoffGoal = require('./TakeoffGoal.js');
let LandingGoal = require('./LandingGoal.js');
let PoseResult = require('./PoseResult.js');
let PoseFeedback = require('./PoseFeedback.js');
let PoseActionGoal = require('./PoseActionGoal.js');
let PoseActionResult = require('./PoseActionResult.js');
let PoseGoal = require('./PoseGoal.js');

module.exports = {
  RC: RC,
  MotorCommand: MotorCommand,
  HeightCommand: HeightCommand,
  PositionXYCommand: PositionXYCommand,
  HeadingCommand: HeadingCommand,
  AttitudeCommand: AttitudeCommand,
  VelocityZCommand: VelocityZCommand,
  RawRC: RawRC,
  Compass: Compass,
  RawMagnetic: RawMagnetic,
  ThrustCommand: ThrustCommand,
  RuddersCommand: RuddersCommand,
  MotorStatus: MotorStatus,
  Supply: Supply,
  YawrateCommand: YawrateCommand,
  ControllerState: ControllerState,
  VelocityXYCommand: VelocityXYCommand,
  Altimeter: Altimeter,
  RawImu: RawImu,
  MotorPWM: MotorPWM,
  ServoCommand: ServoCommand,
  TakeoffFeedback: TakeoffFeedback,
  LandingResult: LandingResult,
  LandingAction: LandingAction,
  PoseAction: PoseAction,
  LandingFeedback: LandingFeedback,
  TakeoffActionGoal: TakeoffActionGoal,
  PoseActionFeedback: PoseActionFeedback,
  TakeoffActionFeedback: TakeoffActionFeedback,
  LandingActionResult: LandingActionResult,
  TakeoffAction: TakeoffAction,
  LandingActionFeedback: LandingActionFeedback,
  LandingActionGoal: LandingActionGoal,
  TakeoffResult: TakeoffResult,
  TakeoffActionResult: TakeoffActionResult,
  TakeoffGoal: TakeoffGoal,
  LandingGoal: LandingGoal,
  PoseResult: PoseResult,
  PoseFeedback: PoseFeedback,
  PoseActionGoal: PoseActionGoal,
  PoseActionResult: PoseActionResult,
  PoseGoal: PoseGoal,
};
