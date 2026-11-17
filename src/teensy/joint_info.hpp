#pragma once

#include <cmath>

constexpr int GRIPPER_PWM_MIN = 1000;
constexpr int GRIPPER_PWM_MAX = 2000;

//constexpr int GRIPPER_ANG_INIT = 0.0; 
constexpr float GRIPPER_ANG_MIN = 0.0;
constexpr float GRIPPER_ANG_MAX = 90.0;

constexpr float GRIPPER_ANG_CLOSE = 12.0;
constexpr float GRIPPER_ANG_OPEN = 30.0;

constexpr float GRIPPER_RATE_DEGPS = 20;
constexpr float GRIPPER_RESOLUTION_DEG = 1.0;

constexpr int numJoints = 5;

constexpr float gearReduction[numJoints] = {8.0, 10.0, 20.0, 4.0,  1.0};
constexpr int stepsPerRev[numJoints]     = {6400, 6400, 6400, 6400, 6400};

constexpr int minStepDelay[numJoints]    = {100,  100,  100,  100,  100};
constexpr float jointAccel[numJoints] = {0.03, 0.05, 0.1, 0.1, 0.1};

constexpr float lowerLimitRad[numJoints] = {-M_PI/4, -M_PI/4, -M_PI/4, -M_PI/4, -M_PI/4};
constexpr float upperLimitRad[numJoints] = {M_PI/4, M_PI/4, M_PI/4, M_PI/4, M_PI/4};

const int encoderAngleHomingStepTargets[numJoints] = {2082, 2039, 2838, 2010, 1983};
const int encoderReverseRange[numJoints] = {1, 0, 0, 0, 0};
const float encoderAngleCalibrationOffsetDeg[numJoints] = {183.0, 88.522, 296.5, 87.1, 0.0};
const float encoderAngleCalibrationOffset[numJoints] = {3.193953, 1.545000, 5.174901, 1.520182, 0.000000};

// Tune these for the physical switch positions and motor wiring before use.

// for 3200 spr:
// const int stepsToMoveAfterLimitSwitchClick[numJoints] = {10285, 5259, 13920, 5643, 1000};
// const int homingStepDelay[numJoints] = {2000, 2000, 800, 1000, 1000};

// for 6400 spr:
const int stepsToMoveAfterLimitSwitchClick[numJoints] = {20570, 9860, 27803, 11442, 2000};
const int homingStepDelay[numJoints] = {1000, 1000, 400, 500, 500};

const int limitSwitchSeekSigns[numJoints] = {1, 1, 1, 1, 1};

struct JointInfo
{
  float gearReduction;
  int stepsPerRev;

  int delay0;
  int minStepDelay;

  float stepsPerRad;
  float radsPerStep;

  float stepsPerDeg;
  float degsPerStep;

  float upperLimitStep;
  float lowerLimitStep;

  float upperLimitRad;
  float lowerLimitRad; 

  
};