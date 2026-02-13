#pragma once


#define serialBufferLength 10
#define serialBufferNumBytes 32


enum class serialCommand : int
{
  HOME = 0,
  TGT_ANGLES_JOINT_SPACE_ACCEL = 1,
  TGT_POSITION_JOINT_SPACE_ACCEL = 2,
  DELTA_ANGLES_JOINT_SPACE_ACCEL = 3,
  DELTA_POSITION_JOINT_SPACE_ACCEL = 4
};