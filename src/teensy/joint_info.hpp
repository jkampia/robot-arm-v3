#pragma once

constexpr int numJoints = 5;

constexpr float limbLengths[numJoints] = {100,  300,  250,  50,   50};

constexpr float gearReduction[numJoints] = {10.0, 10.0, 20.0, 5.0,  1.0};
constexpr int stepsPerRev[numJoints]     = {3200, 3200, 3200, 3200, 3200};
constexpr int minStepDelay[numJoints]    = {20,  20,  20,  20,  20};

struct JointInfo
{
  float gearReduction;
  int stepsPerRev;
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