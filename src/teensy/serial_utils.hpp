#pragma once

#include <string>


#define serialBufferLength 10
#define serialBufferNumBytes 32


enum class serialCommand : int
{
  HOME = 0,

  TGT_ANGLES_JOINT_SPACE = 1,
  TGT_POSITION_JOINT_SPACE = 2, 

  TGT_ANGLES_JOINT_SPACE_ACCEL = 3,
  TGT_POSITION_JOINT_SPACE_ACCEL = 4,

  DELTA_ANGLES_JOINT_SPACE_ACCEL = 5,
  DELTA_POSITION_JOINT_SPACE_ACCEL = 6,

  ENCODER_ANGLE_UPDATE = 7,
  REQUEST_ENCODER_ANGLE_UPDATE = 8,


  GRIPPER_SET_ANGLE = 9,
  GRIPPER_CLOSE = 10,
  GRIPPER_OPEN = 11,

  CONFIRM_RECEIVE = 12,

  RETURN = 13,

  OVERRIDE_CALIBRATION_SAFETY = 14,

  TGT_POSITION_LINE_PATH = 15,
  TGT_POSITION_LINE_PATH_ACCEL = 16,

  DELTA_POSITION_LINE_PATH = 17,
  DELTA_POSITION_LINE_PATH_ACCEL = 18,
};

inline std::string serialCommandToString(serialCommand command)
{
  switch (command)
  {
    case serialCommand::HOME: return "HOME";
    case serialCommand::TGT_ANGLES_JOINT_SPACE: return "TGT_ANGLES_JOINT_SPACE";
    case serialCommand::TGT_POSITION_JOINT_SPACE: return "TGT_POSITION_JOINT_SPACE";
    case serialCommand::TGT_ANGLES_JOINT_SPACE_ACCEL: return "TGT_ANGLES_JOINT_SPACE_ACCEL";
    case serialCommand::TGT_POSITION_JOINT_SPACE_ACCEL: return "TGT_POSITION_JOINT_SPACE_ACCEL";
    case serialCommand::DELTA_ANGLES_JOINT_SPACE_ACCEL: return "DELTA_ANGLES_JOINT_SPACE_ACCEL";
    case serialCommand::DELTA_POSITION_JOINT_SPACE_ACCEL: return "DELTA_POSITION_JOINT_SPACE_ACCEL";
    case serialCommand::ENCODER_ANGLE_UPDATE: return "ENCODER_ANGLE_UPDATE";
    case serialCommand::REQUEST_ENCODER_ANGLE_UPDATE: return "REQUEST_ENCODER_ANGLE_UPDATE";
    case serialCommand::GRIPPER_SET_ANGLE: return "GRIPPER_SET_ANGLE";
    case serialCommand::GRIPPER_CLOSE: return "GRIPPER_CLOSE";
    case serialCommand::GRIPPER_OPEN: return "GRIPPER_OPEN";
    case serialCommand::CONFIRM_RECEIVE: return "CONFIRM_RECEIVE";
    case serialCommand::RETURN: return "RETURN";
    case serialCommand::OVERRIDE_CALIBRATION_SAFETY: return "OVERRIDE_CALIBRATION_SAFETY";
    case serialCommand::TGT_POSITION_LINE_PATH: return "TGT_POSITION_LINE_PATH";
    case serialCommand::TGT_POSITION_LINE_PATH_ACCEL: return "TGT_POSITION_LINE_PATH_ACCEL";
    case serialCommand::DELTA_POSITION_LINE_PATH: return "DELTA_POSITION_LINE_PATH";
    case serialCommand::DELTA_POSITION_LINE_PATH_ACCEL: return "DELTA_POSITION_LINE_PATH_ACCEL";
    default: return "UNKNOWN";
  }
}

// Commands currently handled by parseCommand.
inline bool commandSupported(serialCommand command)
{
  switch (command)
  {
    case serialCommand::HOME:
    case serialCommand::RETURN:
    case serialCommand::OVERRIDE_CALIBRATION_SAFETY:
    case serialCommand::TGT_ANGLES_JOINT_SPACE:
    case serialCommand::TGT_ANGLES_JOINT_SPACE_ACCEL:
    case serialCommand::TGT_POSITION_JOINT_SPACE_ACCEL:
    case serialCommand::DELTA_ANGLES_JOINT_SPACE_ACCEL:
    case serialCommand::TGT_POSITION_LINE_PATH:
    case serialCommand::GRIPPER_SET_ANGLE:
    case serialCommand::GRIPPER_CLOSE:
    case serialCommand::GRIPPER_OPEN:
      return true;

    default:
      return false;
  }
}
