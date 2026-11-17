#pragma once

#include <string>


#define serialBufferLength 10
#define serialBufferNumBytes 32


enum class serialCommand : int
{
  HOME = 0,
  RETURN,
  CONFIRM_RECEIVE, // Report: <CONFIRM_RECEIVE:originating_command>
  OVERRIDE_CALIBRATION_SAFETY,
  MOVE_FAILED, // Report: <MOVE_FAILED:originating_command:reason>
  HOME_J4,
  DISABLE_ALL,
  DISABLE_JOINT,
  ENABLE_ALL,
  ENABLE_JOINT,

  JOINT_ANGLE_UPDATE,
  REQUEST_JOINT_ANGLE_UPDATE,

  ENCODER_STEPS_UPDATE,
  REQUEST_ENCODER_STEPS_UPDATE,

  GRIPPER_SET_ANGLE,
  GRIPPER_CLOSE,
  GRIPPER_OPEN,

  TGT_ANGLES_JOINT_SPACE,
  TGT_POSITION_JOINT_SPACE,

  TGT_ANGLES_JOINT_SPACE_ACCEL,
  TGT_POSITION_JOINT_SPACE_ACCEL,

  DELTA_ANGLES_JOINT_SPACE_ACCEL,
  DELTA_POSITION_JOINT_SPACE_ACCEL,

  TGT_POSITION_LINE_PATH,
  TGT_POSITION_LINE_PATH_ACCEL,

  DELTA_POSITION_LINE_PATH,
  DELTA_POSITION_LINE_PATH_ACCEL,
};

// All declared commands, including reports and commands not yet implemented.
inline std::string serialCommandToString(serialCommand command)
{
  switch (command)
  {
    case serialCommand::HOME: return "HOME";
    case serialCommand::RETURN: return "RETURN";
    case serialCommand::CONFIRM_RECEIVE: return "CONFIRM_RECEIVE";
    case serialCommand::OVERRIDE_CALIBRATION_SAFETY: return "OVERRIDE_CALIBRATION_SAFETY";
    case serialCommand::MOVE_FAILED: return "MOVE_FAILED";
    case serialCommand::HOME_J4: return "HOME_J4";
    case serialCommand::JOINT_ANGLE_UPDATE: return "JOINT_ANGLE_UPDATE";
    case serialCommand::REQUEST_JOINT_ANGLE_UPDATE: return "REQUEST_JOINT_ANGLE_UPDATE";
    case serialCommand::ENCODER_STEPS_UPDATE: return "ENCODER_STEPS_UPDATE";
    case serialCommand::REQUEST_ENCODER_STEPS_UPDATE: return "REQUEST_ENCODER_STEPS_UPDATE";
    case serialCommand::DISABLE_ALL: return "DISABLE_ALL";
    case serialCommand::DISABLE_JOINT: return "DISABLE_JOINT";
    case serialCommand::ENABLE_ALL: return "ENABLE_ALL";
    case serialCommand::ENABLE_JOINT: return "ENABLE_JOINT";
    case serialCommand::GRIPPER_SET_ANGLE: return "GRIPPER_SET_ANGLE";
    case serialCommand::GRIPPER_CLOSE: return "GRIPPER_CLOSE";
    case serialCommand::GRIPPER_OPEN: return "GRIPPER_OPEN";
    case serialCommand::TGT_ANGLES_JOINT_SPACE: return "TGT_ANGLES_JOINT_SPACE";
    case serialCommand::TGT_POSITION_JOINT_SPACE: return "TGT_POSITION_JOINT_SPACE";
    case serialCommand::TGT_ANGLES_JOINT_SPACE_ACCEL: return "TGT_ANGLES_JOINT_SPACE_ACCEL";
    case serialCommand::TGT_POSITION_JOINT_SPACE_ACCEL: return "TGT_POSITION_JOINT_SPACE_ACCEL";
    case serialCommand::DELTA_ANGLES_JOINT_SPACE_ACCEL: return "DELTA_ANGLES_JOINT_SPACE_ACCEL";
    case serialCommand::DELTA_POSITION_JOINT_SPACE_ACCEL: return "DELTA_POSITION_JOINT_SPACE_ACCEL";
    case serialCommand::TGT_POSITION_LINE_PATH: return "TGT_POSITION_LINE_PATH";
    case serialCommand::TGT_POSITION_LINE_PATH_ACCEL: return "TGT_POSITION_LINE_PATH_ACCEL";
    case serialCommand::DELTA_POSITION_LINE_PATH: return "DELTA_POSITION_LINE_PATH";
    case serialCommand::DELTA_POSITION_LINE_PATH_ACCEL: return "DELTA_POSITION_LINE_PATH_ACCEL";
    default: return "UNKNOWN";
  }
}

// Commands currently handled by parseCommand (subject to its calibration guard).
inline bool commandSupported(serialCommand command)
{
  switch (command)
  {
    case serialCommand::HOME:
    case serialCommand::REQUEST_ENCODER_STEPS_UPDATE:
    case serialCommand::HOME_J4:
    case serialCommand::RETURN:
    case serialCommand::OVERRIDE_CALIBRATION_SAFETY:
    case serialCommand::DISABLE_ALL:
    case serialCommand::DISABLE_JOINT:
    case serialCommand::ENABLE_ALL:
    case serialCommand::ENABLE_JOINT:
    case serialCommand::TGT_ANGLES_JOINT_SPACE:
    case serialCommand::TGT_ANGLES_JOINT_SPACE_ACCEL:
    case serialCommand::TGT_POSITION_JOINT_SPACE_ACCEL:
    case serialCommand::DELTA_ANGLES_JOINT_SPACE_ACCEL:
    case serialCommand::TGT_POSITION_LINE_PATH:
    case serialCommand::TGT_POSITION_LINE_PATH_ACCEL:
    case serialCommand::GRIPPER_SET_ANGLE:
    case serialCommand::GRIPPER_CLOSE:
    case serialCommand::GRIPPER_OPEN:
      return true;

    default:
      return false;
  }
}
