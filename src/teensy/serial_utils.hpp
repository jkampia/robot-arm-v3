#pragma once


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
  DELTA_POSITION_JOINT_SPACE_ACCEL = 6
  
};