#pragma once

constexpr int numJoints = 5;

constexpr float limbLengths[numJoints] = {100,  300,  250,  50,   50};

constexpr float gearReduction[numJoints] = {10.0, 10.0, 20.0, 5.0,  1.0};
constexpr int stepsPerRev[numJoints]     = {3200, 3200, 3200, 3200, 3200};

constexpr int minStepDelay[numJoints]    = {60,  20,  20,  20,  20};

constexpr float jointAccel[numJoints] = {0.4, 0.2, 0.2, 0.2, 0.2};

constexpr float lowerLimitRad[numJoints] = {-M_PI/4, -M_PI/4, -M_PI/4, -M_PI/4, -M_PI/4};
constexpr float upperLimitRad[numJoints] = {M_PI/4, M_PI/4, M_PI/4, M_PI/4, M_PI/4};

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