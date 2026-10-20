// Written by Jonathan Kampia
// jonathankampia@gmail.com

// stepper motor wiring order: black-green-red-blue

#include "joint_info.hpp"
#include "kinematics.hpp"
#include "pin_defs.hpp"
#include "print_utils.hpp"
#include "math_utils.hpp"
#include "serial_utils.hpp"

#include <cmath>
#include <array>
#include <climits>
#include <vector>

#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>

Servo gripper;
AS5600 encoder;


enum class specialMode : int
{
  NONE = 0,
  GET_ENCODER_HOME_STEPS = 1
};

specialMode GLOBAL_MODE = specialMode::NONE;
//specialMode GLOBAL_MODE = specialMode::GET_ENCODER_HOME_STEPS;

bool initial_calibration_complete = false;


/* joint stuff */
constexpr float homeJointAngles[numJoints] = {0.0, PI/2, -PI/2, PI/2, 0.0};
constexpr float returnJointAngles[numJoints] = {0.0, PI/2, -PI/2, PI/2, 0.0};

int currentJointSteps[numJoints];
float currentJointAngles[numJoints];

// object containing XYZ of each joint + origin & tooltip
kinematics::JointCoordinates currentJointPositions {};

const int encoderAngleHomingStepTargets[numJoints] = {2082, 2039, 2838, 2010, 0};

const int encoderReverseRange[numJoints] = {1, 0, 0, 0, 0};
const float encoderAngleCalibrationOffsetDeg[numJoints] = {183.0, 88.522, 296.5, 87.1, 0.0};
const float encoderAngleCalibrationOffset[numJoints] = {3.193953, 1.545000, 5.174901, 1.520182, 0.000000};


float currentEncoderAnglesRaw[numJoints];
float currentEncoderAngles[numJoints];

uint16_t currentEncoderSteps[numJoints];

int homingFlags[numJoints] = {1, 1, 1, 1, 0};
bool encoderAvailable[numJoints] = {false, false, false, false, false};

int enaFlags[numJoints] = {0, 0, 0, 0, 0};

JointInfo jointInfo[numJoints];

std::vector<uint16_t> jointDelays[numJoints];


/* AS5600 encoders on TCA9548A mux */
constexpr uint8_t TCA9548A_ADDR = 0x70;
constexpr uint8_t encoderMuxChannels[numJoints] = {2, 3, 4, 5, 6};


/* serial */
String serialBuffer[serialBufferLength];
bool newCommand = false;



bool led_state = LOW;
float led_period_ms = 1000;
float led_on_time_ms = 200;
uint32_t last_led_cycle_time_ms = 0; // ms, arduino mills() returns uint32_t

void setBuiltinLEDRate(const float rate_hz, const float duty_cycle = 0.2)
{
  led_period_ms = 1000 / rate_hz;
  led_on_time_ms = led_period_ms * duty_cycle;
}

/*
  pulse built in LED
*/
void asyncBlinkBuiltinLED()
{
  uint32_t elapsed = millis() - last_led_cycle_time_ms;

  if (elapsed < led_on_time_ms && !led_state)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    led_state = HIGH;
  }

  else if (elapsed >= led_on_time_ms && led_state)
  {
    digitalWrite(LED_BUILTIN, LOW);
    led_state = LOW;
  }

  else if (elapsed > led_period_ms && !led_state)
  {
    last_led_cycle_time_ms = millis();
  }
}


/*
Reads messages in the format: <val1:val2:val3:Xxx>
assigns to global string array serialBuffer
*/
void fastSerialRead(const bool debug = false)
{
  static bool receiving = false;
  static int bufferIndex = 0;
  while (Serial.available() > 0)
  {
    const char inbyte = Serial.read();
    if (inbyte == '<')
    {
      for (int i = 0; i < serialBufferLength; i++) serialBuffer[i] = "";
      bufferIndex = 0;
      receiving = true;
      continue;
    }
    if (!receiving) continue;
    if (inbyte == '>')
    {
      receiving = false;
      newCommand = true;
      if (debug) Serial.println("Received complete command");
      break;
    }
    if (inbyte == ':')
    {
      if (++bufferIndex >= serialBufferLength)
      {
        receiving = false;
        Serial.println("Message has too many segments");
      }
      continue;
    }
    if (serialBuffer[bufferIndex].length() >= serialBufferNumBytes)
    {
      receiving = false;
      Serial.println("Message segment too long");
      continue;
    }
    serialBuffer[bufferIndex] += inbyte;
  }
}


/*
  Pre-calculates some constants based on robot arm parameters
*/
void precalculateJointInfo()
{
  for (int i = 0; i < numJoints; i++)
  {
    jointInfo[i].stepsPerRev = stepsPerRev[i];
    jointInfo[i].gearReduction = gearReduction[i];
    jointInfo[i].minStepDelay = minStepDelay[i];

    int actualStepsPerRev = jointInfo[i].stepsPerRev * jointInfo[i].gearReduction;

    const float revsPerRad = 1/(2*PI);

    jointInfo[i].stepsPerRad = actualStepsPerRev * revsPerRad;
    jointInfo[i].radsPerStep = 1 / jointInfo[i].stepsPerRad;

    const float revsPerDeg = 1/360.0;

    jointInfo[i].stepsPerDeg = actualStepsPerRev * revsPerDeg;
    jointInfo[i].degsPerStep = 1 /  jointInfo[i].stepsPerDeg;

    jointInfo[i].lowerLimitRad = lowerLimitRad[i];
    jointInfo[i].upperLimitRad = upperLimitRad[i];

    /* https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf */
    const float t1 = 500 * 1e-6;
    const float angle = 1;
    const float accel = jointAccel[i];
    jointInfo[i].delay0 = 1/t1 * sqrt(2 * angle / accel) * 0.67703;
  }
}






bool checkInitialCalibrationComplete()
{
  if (!initial_calibration_complete)
  {
    Serial.println("Command denied: robot has not been calibrated yet!");
  }
  return initial_calibration_complete;
}



/*
Set directions for all joints
Currently using digitalWriteFast so each dir pin has to be a compile time static obj
*/
void setJointDirections(const int *jointSigns)
{
  if (jointSigns[0] < 0) digitalWriteFast(dir0, LOW);
  else digitalWriteFast(dir0, HIGH);
  if (jointSigns[1] < 0) digitalWriteFast(dir1, HIGH);
  else digitalWriteFast(dir1, LOW);
  if (jointSigns[2] < 0) digitalWriteFast(dir2, HIGH);
  else digitalWriteFast(dir2, LOW);
  if (jointSigns[3] < 0) digitalWriteFast(dir3, HIGH);
  else digitalWriteFast(dir3, LOW);
  if (jointSigns[4] < 0) digitalWriteFast(dir4, HIGH);
  else digitalWriteFast(dir4, LOW);
}




void parseEnaFlags()
{
  for (int i = 0; i < numJoints; i++)
  {
    digitalWrite(ena[i], enaFlags[i]);
  }
}






/*
Execute a joint-space movement, returning false if validation or timing fails.

FUNCTION PARAMS
desiredJointAngles: array containing each joint's desired angle
implementAcceleration: whether or not to use acceleration S-curve or static speed, default true
*/
bool jogJoints(const float *desiredJointAngles, bool implementAcceleration = true, const int movementTimeUs = 0)
{
  Serial.println("\nNEW JOG:");
  printArray("Current joint angles: ", currentJointAngles, numJoints);
  printArray("Desired joint angles: ", desiredJointAngles, numJoints);

  // check validity of movement -- this needs to be fixed
  /*
  if (!checkAgainstJointLimits(desiredJointAngles))
  {
    Serial.println("Movement invalid");
    return false;
  }
  */

  for (int i = 0; i < numJoints; i++)
  {
    const double steps = fabs(double(desiredJointAngles[i]) - currentJointAngles[i]) * jointInfo[i].stepsPerRad;
    if (!std::isfinite(steps) || steps > INT_MAX)
    {
      Serial.println("Invalid joint target");
      return false;
    }
  }

  // calculate steps to move to satisfy angle delta
  int stepsToMove[numJoints];
  int jointSigns[numJoints];
  for (int i = 0; i < numJoints; i++)
  {
    float diff = desiredJointAngles[i] - currentJointAngles[i];
    stepsToMove[i] = fabsf(diff) * jointInfo[i].stepsPerRad;
    jointSigns[i] = sign(diff);
  }

  printArray("Steps to move: ",        stepsToMove,        numJoints);
  printArray("Joint signs: ",           jointSigns,         numJoints);

  uint32_t startTimeUs = micros();

  // do the thing!!
  if (implementAcceleration)
  {
    if (!goStepsAccel(stepsToMove, jointSigns)) return false;
  }
  else
  {
    if (!goStepsNoAccel(stepsToMove, jointSigns, movementTimeUs)) return false;
  }

  uint32_t stopTimeUs = micros();
  float elapsedTimeS = float(stopTimeUs - startTimeUs) / 1000000;

  printArray("Final joint angles: ", currentJointAngles, numJoints);
  printArray("Final joint steps: ",  currentJointSteps,  numJoints);
  Serial.println("Jog completed in " + String(elapsedTimeS) + " s");

  sendRobotAngleUpdate();
  return true;

}



bool checkFinished(const int* stepCount, const int* stepsToMove, bool* jointEnableFlags)
{
  int activeCount = 0;

  for (int i = 0; i < numJoints; i++)
  {
    if (stepCount[i] >= stepsToMove[i])
    {
      // this joint is done
      jointEnableFlags[i] = false;
    }
    activeCount += jointEnableFlags[i];
  }

  //printArray("Joint enable flags: ", jointEnableFlags, numJoints);

  return activeCount == 0; // will only return true if all joints are 0 / false
};




void doRawSteps(const int* stepsToMove, const int* jointSigns, const bool implementAcceleration)
{
  setJointDirections(jointSigns);

  int stepCount[numJoints];
  uint32_t refTime[numJoints];
  bool pulseHigh[numJoints] = {};
  bool jointEnableFlags[numJoints];
  for (int i = 0; i < numJoints; i++)
  {
    stepCount[i] = 0;
    jointEnableFlags[i] = true;
    refTime[i] = micros();
  }

  while (!checkFinished(stepCount, stepsToMove, jointEnableFlags))
  {
    // unfortunately this cannot be made into a for loop due to the pulse pins
    // needing to be static compile-time objects

    // joint 0
    if (stepsToMove[0] != 0 && jointEnableFlags[0])
    {
      auto delay = jointDelays[0].front();
      if (implementAcceleration) { delay = jointDelays[0][stepCount[0]]; }
      uint32_t now = micros();
      uint32_t dt = now - refTime[0];
      uint32_t half = delay >> 1;

      if (!pulseHigh[0] && dt >= half)
      {
        digitalWriteFast(pul0, HIGH);
        pulseHigh[0] = true;
        // Preserve a full HIGH half-period even when this loop runs late.
        refTime[0] = now - half;
        //Serial.println("H");
      }
      else if (pulseHigh[0] && dt >= delay)
      {
        digitalWriteFast(pul0, LOW);
        pulseHigh[0] = false;
        //Serial.println("L");
        stepCount[0]++;
        refTime[0] = now;
      }
    }

    // joint 1
    if (stepsToMove[1] != 0 && jointEnableFlags[1])
    {
      auto delay = jointDelays[1].front();
      if (implementAcceleration) { delay = jointDelays[1][stepCount[1]]; }
      uint32_t now = micros();
      uint32_t dt = now - refTime[1];
      uint32_t half = delay >> 1;

      if (!pulseHigh[1] && dt >= half)
      {
        digitalWriteFast(pul1, HIGH);
        pulseHigh[1] = true;
        // Preserve a full HIGH half-period even when this loop runs late.
        refTime[1] = now - half;
      }
      else if (pulseHigh[1] && dt >= delay)
      {
        digitalWriteFast(pul1, LOW);
        pulseHigh[1] = false;
        stepCount[1]++;
        refTime[1] = now;
      }
    }

    // joint 2
    if (stepsToMove[2] != 0 && jointEnableFlags[2])
    {
      auto delay = jointDelays[2].front();
      if (implementAcceleration) { delay = jointDelays[2][stepCount[2]]; }
      uint32_t now = micros();
      uint32_t dt = now - refTime[2];
      uint32_t half = delay >> 1;

      if (!pulseHigh[2] && dt >= half)
      {
        digitalWriteFast(pul2, HIGH);
        pulseHigh[2] = true;
        // Preserve a full HIGH half-period even when this loop runs late.
        refTime[2] = now - half;
      }
      else if (pulseHigh[2] && dt >= delay)
      {
        digitalWriteFast(pul2, LOW);
        pulseHigh[2] = false;
        stepCount[2]++;
        refTime[2] = now;
      }
    }

    // joint 3
    if (stepsToMove[3] != 0 && jointEnableFlags[3])
    {
      auto delay = jointDelays[3].front();
      if (implementAcceleration) { delay = jointDelays[3][stepCount[3]]; }
      uint32_t now = micros();
      uint32_t dt = now - refTime[3];
      uint32_t half = delay >> 1;

      if (!pulseHigh[3] && dt >= half)
      {
        digitalWriteFast(pul3, HIGH);
        pulseHigh[3] = true;
        // Preserve a full HIGH half-period even when this loop runs late.
        refTime[3] = now - half;
      }
      else if (pulseHigh[3] && dt >= delay)
      {
        digitalWriteFast(pul3, LOW);
        pulseHigh[3] = false;
        stepCount[3]++;
        refTime[3] = now;
      }
    }

    // joint 4
    if (stepsToMove[4] != 0 && jointEnableFlags[4])
    {
      auto delay = jointDelays[4].front();
      if (implementAcceleration) { delay = jointDelays[4][stepCount[4]]; }
      uint32_t now = micros();
      uint32_t dt = now - refTime[4];
      uint32_t half = delay >> 1;

      if (!pulseHigh[4] && dt >= half)
      {
        digitalWriteFast(pul4, HIGH);
        pulseHigh[4] = true;
        // Preserve a full HIGH half-period even when this loop runs late.
        refTime[4] = now - half;
      }
      else if (pulseHigh[4] && dt >= delay)
      {
        digitalWriteFast(pul4, LOW);
        pulseHigh[4] = false;
        stepCount[4]++;
        refTime[4] = now;
      }
    }
  }

  // add # of steps moved to current joint step position
  // apply movement to joint angles based on actual steps taken
  // not perceived angular movement
  // assumes no skipped steps
  for (int i = 0; i < numJoints; i++)
  {
    currentJointSteps[i] += stepCount[i] * jointSigns[i];
    currentJointAngles[i] = currentJointSteps[i] * jointInfo[i].radsPerStep;
  }
  currentJointPositions = kinematics::solveFK(currentJointAngles);
}



// resets reference point for robot arm -- be very careful when calling this, as it will
// screw up the robot's current kinematic reference
void zeroAllJoints()
{
  for (int i = 0; i < numJoints; i++)
  {
    currentJointSteps[i] = int(homeJointAngles[i] * jointInfo[i].stepsPerRad);
    currentJointAngles[i] = currentJointSteps[i] * jointInfo[i].radsPerStep;
  }

  currentJointPositions = kinematics::solveFK(currentJointAngles);

  printArray("Zeroed joint steps: ", currentJointSteps, numJoints);
  printArray("Zeroed joint angles: ", currentJointAngles, numJoints);
}



// returns true if all command angles are valid & do not exceed joint limits
// commandAngles should be in radians
bool checkAgainstJointLimits(const float* commandAngles)
{
  for (int i = 0; i < numJoints; i++)
  {
    if (commandAngles[i] > jointInfo[i].upperLimitRad || commandAngles[i] < jointInfo[i].lowerLimitRad)
    {
      return false;
    }
  }
  return true;
}



// checks a lot of stuff and fixes step delays if necessary, then passes to doRawSteps() function
bool goStepsNoAccel(const int* stepsToMove, const int* jointSigns, const int movementTimeUs)
{
  if (movementTimeUs <= 0)
  {
    Serial.print("Invalid movement time provided: " + String(movementTimeUs));
    return false;
  }

  // joint can skip steps if delay is < 100 us, and cannot hold value > 65535
  const int min_step_delay = 100;
  const int max_step_delay = 65535;

  // Validate before starting any joint, and leave inactive delay vectors empty.
  for (int i = 0; i < numJoints; i++)
  {
    const bool stepcount_negative = stepsToMove[i] < 0;
    const bool stepcount_positive = stepsToMove[i] > 0;
    const bool no_joint_sign = jointSigns[i] != -1 && jointSigns[i] != 1;
    const int pulse_period = stepcount_positive ? movementTimeUs / stepsToMove[i] : 0;
    const bool pulse_period_invalid = stepcount_positive && pulse_period < 2;
    const bool pulse_period_too_large = stepcount_positive && pulse_period > max_step_delay;

    const bool failure_conditions_if_positive = stepcount_positive && (no_joint_sign || pulse_period_invalid || pulse_period_too_large);

    if (stepcount_negative || failure_conditions_if_positive)
    {
      Serial.println("Invalid step count, direction, or pulse period");
      return false;
    }
  }

  for (int i = 0; i < numJoints; i++)
  {
    jointDelays[i].clear();
    if (stepsToMove[i] > 0)
    {
      jointDelays[i].push_back(movementTimeUs / stepsToMove[i]);
    }
  }

  int shortest_period = max_step_delay; // high initial value for comparisons
  bool pulse_period_too_short = false;

  for (int i = 0; i < numJoints; i++)
  {
    if (!jointDelays[i].empty() && jointDelays[i].front() < min_step_delay)
    {
      pulse_period_too_short = true;
      shortest_period = min(shortest_period, int(jointDelays[i].front()));
    }
  }

  if (pulse_period_too_short)
  {
    // Apply one common scale factor so all joints keep their relative timing.
    // Integer ceiling ensures the shortest period reaches at least the minimum.

    const float scale_factor = float(min_step_delay) / shortest_period;

    for (int i = 0; i < numJoints; i++)
    {
      // check validity of delays post-scalefactor first
      for (const auto delay : jointDelays[i])
      {
        // round up to avoid dipping below the min delay
        const uint32_t scaled_delay = uint32_t(ceilf(float(delay) * scale_factor));

        if (scaled_delay > max_step_delay)
        {
          Serial.println("Invalid scaling - scaled pulse period exceeds max step delay");
          return false;
        }
      }
    }

    // then apply it
    for (int i = 0; i < numJoints; i++)
    {
      for (auto& delay : jointDelays[i])
      {
        delay = uint16_t(ceilf(float(delay) * scale_factor));
      }
    }
  }

  // once all checks are passed and all delays are scaled, do the movement
  const bool implementAcceleration = false;
  doRawSteps(stepsToMove, jointSigns, implementAcceleration);

  return true;
}



bool goStepsAccel(const int *stepsToMove, const int* jointSigns)
{
  for (int i = 0; i < numJoints; i++)
  {
    if (stepsToMove[i] < 0 || (stepsToMove[i] > 0 && jointSigns[i] != -1 && jointSigns[i] != 1))
    {
      Serial.println("Invalid step count or direction");
      return false;
    }
  }
  float sumTime[numJoints] = {0.0, 0.0, 0.0, 0.0, 0.0};
  //float jointTimeScalar[numJoints] = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < numJoints; i++)
  {
    // wipe vector, maintains its space in memory
    jointDelays[i].clear();
    // populate jointDelays with new delay values
    computeStepDelays(jointInfo[i], abs(stepsToMove[i]), jointDelays[i]);
    // add up all delays to calculate total movement time (microseconds)
    for (const auto& val : jointDelays[i]) { sumTime[i] += val; }
  }

  // find which joint takes the longest, its time, and its index
  float maxTime = 0;
  for (int i = 0; i < numJoints; i++)
  {
    if (sumTime[i] > maxTime)
    {
      maxTime = sumTime[i];
    }
    //Serial.println(sumTime[i]);
  }
  //Serial.println("Max time: " + String(maxTime));

  for (int i = 0; i < numJoints; i++)
  {
    if (sumTime[i] != 0) {
      float jointTimeScalar = maxTime / sumTime[i];
      //Serial.println(jointTimeScalar);
      if (!scaleVector(jointDelays[i], jointTimeScalar))
      {
        Serial.println("Scaled acceleration pulse period exceeds timer capacity");
        return false;
      }
    }
  }

  const bool implementAcceleration = true;
  doRawSteps(stepsToMove, jointSigns, implementAcceleration);
  return true;
}



/*
Magic secret sauce function, calculates step delays required for smooth S-curve
for a single joint

FUNCTION PARAMS
motorIndex: the joint index (0-4)
numSteps: the number of steps the joint has to move
delayVector: the joint delay vector to be populated
*/
void computeStepDelays(const JointInfo jointInfo, const int numSteps, std::vector<uint16_t>& delayVector)
{
  /* https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf */
  //const float t1 = 500 * 1e-6;
  //const float angle = 1;
  //const float accel = 0.1;
  //const float delay0 = 1/t1 * sqrt(2 * angle / accel) * 0.67703;

  //const float delay0 = 2000 * sqrt(2*angle / accel) * 0.67703;

  float d = jointInfo.delay0;
  int n = 0;
  int rampUpStepCount = 0;
  //int totalSteps = 0;

  for (int i = 0; i < numSteps; i++) {
    if (rampUpStepCount == 0) {
      n++;
      d = d - (2 * d) / (4 * n + 1);
      if (d <= jointInfo.minStepDelay) {
        d = jointInfo.minStepDelay;
        rampUpStepCount = i;
        //Serial.println(rampUpStepCount);
      }
      if (i >= numSteps / 2) {
        rampUpStepCount = i;
        //Serial.println(rampUpStepCount);
      }
    }
    else if (i >= numSteps - rampUpStepCount) {
      n--;
      d = (d * (4 * n + 1)) / (4 * n + 1 - 2);
    }

    //Serial.println(uint16_t(d));
    delayVector.push_back(uint16_t(d));
    //Serial.println(delay_array[i]);
  }
}



void jogPositionJointSpace(float* desiredPoseXYZPR)
{
  float desiredAngles[numJoints];
  if (!kinematics::solveIK(desiredPoseXYZPR, desiredAngles)) return;

  const bool implementAcceleration = true;
  jogJoints(desiredAngles, implementAcceleration);
}





const float linepath_resolution_mm = 1.0;
const float linepath_speed_mmps = 200;

void jogPositionLinePath(float* desiredPoseXYZPR)
{
  if (!desiredPoseXYZPR) return;
  for (int j = 0; j < numJoints; j++)
  {
    if (!std::isfinite(desiredPoseXYZPR[j]))
    {
      Serial.println("Invalid line path target");
      return;
    }
  }

  float start[numJoints];
  float delta[numJoints];
  for (int axis = 0; axis < 3; axis++) start[axis] = currentJointPositions.back()[axis];
  start[3] = currentJointAngles[1] + currentJointAngles[2] + currentJointAngles[3] - kinematics::halfPi;
  start[4] = currentJointAngles[4];

  arraySubtract(desiredPoseXYZPR, start, delta, numJoints);
  const float distance = sqrtf(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
  if (distance == 0 && delta[3] == 0 && delta[4] == 0) return;

  // For a pure orientation move, use the tool's angular travel as path length.
  const float path_length = distance > 0 ? distance : fmaxf(fabsf(delta[3]), fabsf(delta[4])) * (limbLengths[3] + limbLengths[4]);

  const float segments = ceilf(path_length / linepath_resolution_mm);
  // Bound planning memory on the microcontroller.
  if (!std::isfinite(segments) || segments > 4096 || segments < 1)
  {
    Serial.println("Line path exceeds planning capacity");
    return;
  }

  const int count = int(segments);

  // Equal-length segments share one duration; goStepsNoAccel handles joint scaling.
  const double duration = ceil(1000000.0 * path_length / count / linepath_speed_mmps);
  if (!std::isfinite(duration) || duration < 1 || duration > INT_MAX) return;
  const int segment_time_us = int(duration);

  // Validate the complete path before executing its joint-angle waypoints.
  std::vector<std::array<float, numJoints>> trajectory;
  trajectory.reserve(count);

  // Preflight every waypoint before issuing any motor pulses
  for (int i = 0; i < count; i++)
  {
    float pose[numJoints];
    std::array<float, numJoints> angles;
    const float fraction = float(i + 1) / count;
    for (int j = 0; j < numJoints; j++)
    {
      pose[j] = i == count - 1 ? desiredPoseXYZPR[j] : start[j] + fraction * delta[j];
    }

    if (!kinematics::solveIK(pose, angles.data())) return;
    trajectory.push_back(angles);
  }

  for (int i = 0; i < count; i++)
  {
    if (!jogJoints(trajectory[i].data(), false, segment_time_us))
    {
      Serial.println("Line path stopped: segment rejected");
      return;
    }
  }

}


float mapGripperAngle(float angle)
{
  return GRIPPER_PWM_MIN + (angle / GRIPPER_ANG_MAX) * (GRIPPER_PWM_MAX - GRIPPER_PWM_MIN);
}


float clampGripperAngle(float angle)
{
  if (angle < GRIPPER_ANG_MIN) return GRIPPER_ANG_MIN;
  if (angle > GRIPPER_ANG_MAX) return GRIPPER_ANG_MAX;
  return angle;
}



void setGripperAngle(const float angle)
{
  // calculate pwm setpoint
  float angle_clamped = clampGripperAngle(angle);
  float pwm_setpt = mapGripperAngle(angle_clamped);

  gripper.writeMicroseconds((int)pwm_setpt);

  Serial.print("Wrote gripper angle: ");
  Serial.print(angle_clamped);
  Serial.print(" -> pwm setpoint: ");
  Serial.print(pwm_setpt);
  Serial.println("us");
}




bool selectEncoderMuxChannel(const uint8_t channel)
{
  if (channel > 7) return false;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(uint8_t(1 << channel));
  bool channelSelected = Wire.endTransmission() == 0;
  delayMicroseconds(10);
  return channelSelected;
}


bool initializeEncoders(const bool debug = false)
{
  bool allEncodersOk = true;

  for (int i = 0; i < numJoints; i++)
  {
    bool encoderOk = selectEncoderMuxChannel(encoderMuxChannels[i]) && encoder.begin();

    encoderAvailable[i] = encoderOk;
    if (!encoderOk)
    {
      allEncodersOk = false;
      if (debug)
      {
        Serial.println("WARNING: AS5600 not detected on TCA9548A channel " + String(encoderMuxChannels[i]));
      }

      // Encoder availability must not disable limit-switch homing; see encoderAvailable.
    }
  }

  return allEncodersOk;
}


/*
Reads 5 AS5600 encoders connected to TCA9548A channels SD2-SD6.
Output angles are in degrees.
*/
bool readEncoderAngles(const bool debug = false)
{
  uint32_t startTimeUs = micros();
  bool allReadsOk = true;

  for (int i = 0; i < numJoints; i++)
  {
    bool readOk = selectEncoderMuxChannel(encoderMuxChannels[i]) && encoder.isConnected();

    if (readOk)
    {
      uint16_t encoderSteps = encoder.rawAngle();
      readOk = encoder.lastError() == AS5600_OK;

      if (readOk)
      {
        float temp = float(encoderSteps);
        if (encoderReverseRange[i])
        {
          temp = 4096.0 - encoderSteps;
        }

        currentEncoderSteps[i] = temp;

        currentEncoderAngles[i] = temp * 2*PI / 4096.0 - encoderAngleCalibrationOffset[i];
      }
    }

    if (!readOk)
    {
      allReadsOk = false;
      /*
      if (debug)
      {
        Serial.println("Failed to read AS5600 on TCA9548A channel " + String(encoderMuxChannels[i]));
      }
      */
    }
  }
  uint32_t elapsedTimeUs = micros() - startTimeUs;

  if (debug)
  {
    printArray("Encoder steps: ", currentEncoderSteps, numJoints);
    Serial.println("Read encoder angles in " + String(elapsedTimeUs) + " us");
  }

  return allReadsOk;
}


void sendRobotAngleUpdate()
{
  Serial.print("<");
  Serial.print(int(serialCommand::ENCODER_ANGLE_UPDATE));

  for (int i = 0; i < numJoints; i++)
  {
    Serial.print(":");
    Serial.print(currentJointAngles[i], 4);
  }

  for (int i = 0; i < numJoints; i++)
  {
    Serial.print(":");
    Serial.print(currentJointSteps[i]);
  }

  Serial.print(">");
  Serial.print("\n");
}



/*
Acts on serialBuffer global array of commands
*/
void parseCommand(const bool debug = false)
{
  if (!newCommand) return;

  // check through string debug commands
  String command = serialBuffer[0];

  serialCommand commandSignature = static_cast<serialCommand>(command.toInt());
  //Serial.println(serialBuffer[0].toInt());


  // only allow the HOME & OVERRIDE command through if calibration is incomplete
  if (!checkInitialCalibrationComplete()
        && commandSignature != serialCommand::HOME
        && commandSignature != serialCommand::OVERRIDE_CALIBRATION_SAFETY)
  {
    newCommand = false;
    return;
  }


  switch(commandSignature)
  {
    case serialCommand::HOME:
    {
      Serial.println("Received HOME command");
      sendReceiveConfirmation(commandSignature);

      homeWithLimitSwitchFeedback();
      break;
    }

    case serialCommand::RETURN:
    {
      Serial.println("Received RETURN command");
      sendReceiveConfirmation(commandSignature);

      returnToHome();
      break;
    }

    case serialCommand::OVERRIDE_CALIBRATION_SAFETY:
    {
      Serial.println("Received OVERRIDE_CALIBRATION_SAFETY command");
      sendReceiveConfirmation(commandSignature);

      initial_calibration_complete = true;
      break;
    }

    case serialCommand::TGT_ANGLES_JOINT_SPACE:
    {
      Serial.println("Received TGT_ANGLES_JOINT_SPACE command");
      sendReceiveConfirmation(commandSignature);

      float desiredAngles[numJoints];
      memcpy(desiredAngles, currentJointAngles, numJoints * sizeof(float));

      for (int i = 0; i < numJoints; i++)
      {
        if (serialBuffer[i+1].length() > 0)
        {
          desiredAngles[i] = serialBuffer[i+1].toFloat();
        }
      }

      const bool implementAcceleration = false;
      const int movementTimeUs = 3000000;
      jogJoints(desiredAngles, implementAcceleration, movementTimeUs);
      break;
    }

    case serialCommand::TGT_ANGLES_JOINT_SPACE_ACCEL:
    {
      Serial.println("Received TGT_ANGLES_JOINT_SPACE_ACCEL command");
      sendReceiveConfirmation(commandSignature);

      float desiredAngles[numJoints];
      // copy over current angles as defaults in case msg is not populated correctly
      memcpy(desiredAngles, currentJointAngles, numJoints * sizeof(float));

      for (int i = 0; i < numJoints; i++)
      {
        if (serialBuffer[i+1].length() > 0)
        {
          desiredAngles[i] = serialBuffer[i+1].toFloat();
        }
      }

      const bool implementAcceleration = true;
      jogJoints(desiredAngles, implementAcceleration);
      break;
    }

    case serialCommand::TGT_POSITION_JOINT_SPACE_ACCEL:
    {
      Serial.println("Received TGT_POSITION_JOINT_SPACE_ACCEL command");
      sendReceiveConfirmation(commandSignature);

      float desiredPoseXYZPR[numJoints];
      bool commandValid = true;

      for (int i = 0; i < numJoints; i++)
      {
        if (serialBuffer[i+1].length() > 0)
        {
          desiredPoseXYZPR[i] = serialBuffer[i+1].toFloat();
        }
        else
        {
          Serial.println("Position command incomplete: arg" + String(i) + " missing");
          commandValid = false;
          break;
        }
      }

      if (!commandValid) break;

      jogPositionJointSpace(desiredPoseXYZPR);

      break;
    }

    case serialCommand::DELTA_ANGLES_JOINT_SPACE_ACCEL:
    {
      Serial.println("Received DELTA_ANGLES_JOINT_SPACE_ACCEL command");
      sendReceiveConfirmation(commandSignature);

      // copy current joint angles
      float desiredAngles[numJoints];
      memcpy(desiredAngles, currentJointAngles, numJoints * sizeof(float));

      for (int i = 0; i < numJoints; i++)
      {
        if (serialBuffer[i+1].length() > 0)
        {
          desiredAngles[i] += serialBuffer[i+1].toFloat();
        }
      }

      const bool implementAcceleration = true;
      jogJoints(desiredAngles, implementAcceleration);

      break;
    }

    case serialCommand::GRIPPER_SET_ANGLE:
    {
      Serial.println("Received GRIPPER_SET_ANGLE command");
      sendReceiveConfirmation(commandSignature);

      float angle = serialBuffer[1].toFloat();
      setGripperAngle(angle);

      break;
    }

    case serialCommand::GRIPPER_CLOSE:
    {
      Serial.println("Received GRIPPER_CLOSE command");
      sendReceiveConfirmation(commandSignature);

      float close_angle = GRIPPER_ANG_MIN;
      setGripperAngle(close_angle);

      break;
    }

    case serialCommand::GRIPPER_OPEN:
    {
      Serial.println("Received GRIPPER_OPEN command");
      sendReceiveConfirmation(commandSignature);

      float open_angle = GRIPPER_ANG_MAX;
      setGripperAngle(open_angle);

      break;
    }

    default:
    {
      Serial.println("Received unknown command signature");
      break;
    }
  }

  newCommand = false;
  Serial.print("\n");
}



// send confirmation to frontend that a given command was received
void sendReceiveConfirmation(serialCommand confirmCommand)
{
  Serial.print("<");
  Serial.print(int(serialCommand::CONFIRM_RECEIVE));
  Serial.print(":");
  Serial.print(int(confirmCommand));
  Serial.print(">");
  Serial.print("\n");
}



void printFreeMemory()
{
  // for Teensy 3.0
  uint32_t stackTop;
  uint32_t heapTop;

  // current position of the stack.
  stackTop = (uint32_t) &stackTop;

  // current position of heap.
  void* hTop = malloc(1);
  heapTop = (uint32_t) hTop;
  free(hTop);

  // The difference is (approximately) the free, available ram.
  Serial.println("Free RAM: " + String(stackTop - heapTop) + String("B"));
}



void disableAllJoints()
{
  for (int i = 0; i < 5; i++)
  {
    enaFlags[i] = 1;
  }

  parseEnaFlags();
}



/*
  Home each joint
*/
void homeWithEncoderFeedback()
{
  int j_idx = 0;
  float tolerance = 1;
  int stepsToMove[numJoints] = {0, 0, 0, 0, 0};
  int jointSigns[numJoints] = {0, 0, 0, 0, 0};

  int movementTimeUs = 400;

  int error;
  int prev_error;

  for (int i = 0; i < numJoints; i++)
  {
    if (!homingFlags[i] || !encoderAvailable[i])
    {
      Serial.println("Skipping joint " + String(i));
      continue;
    } // skip disabled joints

    j_idx = i;

    readEncoderAngles();
    error = encoderAngleHomingStepTargets[j_idx] - currentEncoderSteps[j_idx];
    zeroArray(stepsToMove, numJoints);
    stepsToMove[j_idx] = 1;
    jointSigns[j_idx] = sign(error);

    movementTimeUs = 400;

    Serial.println("Initial Error: " + String(error));

    while (abs(error) > tolerance)
    {
      prev_error = error;

      //float error_norm = abs(error) / 2*PI;
      //movementTimeUs = 400 - 300 * error_norm;

      goStepsNoAccel(stepsToMove, jointSigns, movementTimeUs);

      readEncoderAngles();
      error = encoderAngleHomingStepTargets[j_idx] - currentEncoderSteps[j_idx];
      jointSigns[j_idx] = sign(error);
      Serial.println("Error (steps): " + String(error));

      if (sign(error) != sign(prev_error))
      {
        Serial.println("Error crossed over zero!");
        break;
      }
    }
  }


  for (int i = 0; i < numJoints; i++)
  {
    if (!homingFlags[i] || !encoderAvailable[i]) continue;
    currentJointAngles[i] = homeJointAngles[i];
    currentJointSteps[i] = homeJointAngles[i] * jointInfo[i].stepsPerRad;
  }

  currentJointPositions = kinematics::solveFK(currentJointAngles);

  // rotate j2 -90 degrees
  // only do it if the encoder was detected, to avoid undefined behavior
  if (homingFlags[2] && encoderAvailable[2])
  {
    const float j2_homing_offset = -PI/2;

    float desiredAngles[numJoints];
    memcpy(desiredAngles, currentJointAngles, numJoints * sizeof(float));

    desiredAngles[2] += j2_homing_offset;

    const bool implementAcceleration = true;
    jogJoints(desiredAngles, implementAcceleration);
  }

  initial_calibration_complete = true;

  // slow down LED blink rate to indicate that calibration values have been applied
  // and it is now safe to move the arm
  const float slow_led_rate = 1;
  setBuiltinLEDRate(slow_led_rate);

  Serial.println("Done homing");
  printArray("Current joint angles: ", currentJointAngles, numJoints);
  printArray("Current joint steps: ", currentJointSteps, numJoints);
}



/*
  Limit switches all bridge through Normally Closed (NC), so if one switch is pressed
  it breaks the entire link
  Normally, GND -- GPIO (input pullup), so reading GPIO pulls LOW
  if switch pressed, GND -/- GPIO, so reading GPIO pulls HIGH
*/

bool readLimitSwitchState(const uint32_t numSamples = 5)
{
  const uint32_t sampleCount = numSamples > 0 ? numSamples : 1;
  uint32_t highSamples = 0;
  for (uint32_t i = 0; i < sampleCount; i++)
  {
    if (digitalReadFast(LIMIT_SWITCH_PIN)) highSamples++;
  }

  // HIGH means pressed. Resolve an even split as pressed as well.
  return highSamples >= sampleCount - highSamples;
}

constexpr uint32_t limitSwitchSampleWindowUs = 10000;
constexpr uint32_t limitSwitchMinMatchPercent = 95;

/*
  Limit switches seem to be prone to bouncing when the arm moves, so we need to filter out
  erroneous HIGH (open) readings.
  Function returns true if the percentage of readings over the sample window meet a minimum threshold
*/
bool waitForStableLimitSwitchState(const bool expectedState, const bool reportInvalid = true)
{
  const uint32_t startUs = micros();
  uint32_t totalSamples = 0;
  uint32_t matchingSamples = 0;

  do
  {
    if (readLimitSwitchState() == expectedState) matchingSamples++;
    totalSamples++;
  }
  while (uint32_t(micros() - startUs) < limitSwitchSampleWindowUs);

  const bool valid = uint64_t(matchingSamples) * 100 >= uint64_t(totalSamples) * limitSwitchMinMatchPercent;

  asyncBlinkBuiltinLED();

  if (!valid && reportInvalid)
  {
    // Print after sampling so serial output does not affect the window.
    Serial.print("Limit input: matching samples=");
    Serial.print(matchingSamples);
    Serial.print("/");
    Serial.print(totalSamples);
    Serial.print(", match percent=");
    Serial.print(100.0f * matchingSamples / totalSamples);
    Serial.print(", required percent=");
    Serial.print(limitSwitchMinMatchPercent);
    Serial.print(", expected=");
    Serial.println(expectedState ? "HIGH (open)" : "LOW (closed)");
  }
  return valid;
}



// Tune these for the physical switch positions and motor wiring before use.
const int stepsToMoveAfterLimitSwitchClick[numJoints] = {10285, 5259, 13920, 5643, 1000};
const int limitSwitchSeekSigns[numJoints] = {1, 1, 1, 1, 1};
const int homingStepDelay[numJoints] = {2000, 2000, 800, 1000, 1000};
const uint32_t limitSwitchSeekTimeoutMs = 30000;


//Home each joint expecting limit switch feedback
void homeWithLimitSwitchFeedback()
{
  initial_calibration_complete = false;

  setBuiltinLEDRate(5);

  int stepsToMove[numJoints] = {0};
  int jointSigns[numJoints] = {0};
  int stepsMoved[numJoints] = {0};

  bool allJointsHomed = true;



  for (int i = numJoints-1; i >= 0; i--)
  {
    const int movementTimeUs = homingStepDelay[i];

    if (!homingFlags[i])
    {
      Serial.println("Skipping joint " + String(i));

      if (i != numJoints-1) allJointsHomed = false;

      continue;
    }

    if (stepsToMoveAfterLimitSwitchClick[i] <= 0 || abs(limitSwitchSeekSigns[i]) != 1)
    {
      Serial.println("Homing aborted: invalid joint configuration");
      return;
    }

    // set up movement parameters
    zeroArray(stepsToMove, numJoints);
    stepsToMove[i] = 1;
    jointSigns[i] = limitSwitchSeekSigns[i];

    // Bound seeking by the configured joint travel as well as elapsed time.
    const int maxSeekSteps = int(ceil((jointInfo[i].upperLimitRad - jointInfo[i].lowerLimitRad) * jointInfo[i].stepsPerRad));

    const uint32_t startMs = millis();
    int seekSteps = 0;

    while (true)
    {
      if (/*seekSteps >= maxSeekSteps ||*/ uint32_t(millis() - startMs) >= limitSwitchSeekTimeoutMs)
      {
        Serial.println("Homing aborted: seek limit reached for joint " + String(i));
        return;
      }
      if (readLimitSwitchState())
      {
        // Pause on majority HIGH. A confirmed hit ends seeking; a confirmed LOW
        // means it was a glitch, so retry without resetting the seek timeout.
        if (waitForStableLimitSwitchState(true, false)) break;
        if (!waitForStableLimitSwitchState(false))
        {
          Serial.println("Homing aborted: ambiguous limit input for joint " + String(i));
          return;
        }
        continue;
      }

      goStepsNoAccel(stepsToMove, jointSigns, movementTimeUs);
      seekSteps++;

      // update step count
      stepsMoved[i] += stepsToMove[i];
    }

    // reverse away from the switch to the calibrated home offset
    stepsToMove[i] = stepsToMoveAfterLimitSwitchClick[i];
    jointSigns[i] = -limitSwitchSeekSigns[i];
    goStepsAccel(stepsToMove, jointSigns);

    if (!waitForStableLimitSwitchState(false))
    {
      Serial.println("Homing aborted: limit circuit did not settle closed after backoff");
      return;
    }

    currentJointAngles[i] = homeJointAngles[i];
    currentJointSteps[i] = homeJointAngles[i] * jointInfo[i].stepsPerRad;
    currentJointPositions = kinematics::solveFK(currentJointAngles);
  }

  // Report steps taken to reach the limit switch for each joint
  printArray("Homing steps moved: ", stepsMoved, numJoints);

  // Preserve skipped joint positions and keep motion locked after partial homing.
  if (!allJointsHomed)
  {
    Serial.println("Partial homing complete; calibration remains incomplete");
    return;
  }

  initial_calibration_complete = true;

  setBuiltinLEDRate(1);

  Serial.println("Done homing");
  printArray("Current joint angles: ", currentJointAngles, numJoints);
  printArray("Current joint steps: ", currentJointSteps, numJoints);
}


/*
  return to home position (different from intial homing)
*/
void returnToHome()
{
  float desiredAngles[numJoints];
  memcpy(desiredAngles, returnJointAngles, numJoints * sizeof(float));

  const bool implementAcceleration = true;
  jogJoints(desiredAngles, implementAcceleration);

  Serial.println("Returned to home position");
}





void setup()
{
  Serial.begin(921600);

  Serial.print("\n");
  Serial.println("[SETUP]");
  Serial.print("\n");

  // TCA9548A encoder i2c bus encoder setup

  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();
  Wire.setClock(400000);
  initializeEncoders(true);

  printArray("Homing flags: ", homingFlags, numJoints);

  // set motor output pins

  pinMode(pul0, OUTPUT);
  pinMode(dir0, OUTPUT);
  pinMode(ena0, OUTPUT);

  pinMode(pul1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(ena1, OUTPUT);

  pinMode(pul2, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(ena2, OUTPUT);

  pinMode(pul3, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(ena3, OUTPUT);

  pinMode(pul4, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(ena4, OUTPUT);

  gripper.attach(GRIPPER_PWM_PIN, GRIPPER_PWM_MIN, GRIPPER_PWM_MAX);
  setGripperAngle(GRIPPER_ANG_MIN);

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // set led rate to a fast blink to indicate pre-calibration state
  const float fast_led_rate = 5; // hz
  setBuiltinLEDRate(fast_led_rate);


  // reserve n bytes @ each address of serialBuffer
  for (int i = 0; i < serialBufferLength; i++)
  {
    serialBuffer[i].reserve(serialBufferNumBytes);
  }

  printArray("Link lengths: ", limbLengths, numJoints);
  precalculateJointInfo();

  // reserve memory for joint movement delay storage & copy all joint values into storage
  for (int i = 0; i < numJoints; i++)
  {
    jointDelays[i].reserve(32000);

    currentJointSteps[i] = homeJointAngles[i] * jointInfo[i].stepsPerRad;
    currentJointAngles[i] = homeJointAngles[i];

    parseEnaFlags();
  }
  currentJointPositions = kinematics::solveFK(currentJointAngles);

  printFreeMemory();

  // read encoder angles & assign joint angles
  // readEncoderAngles();

  // debug print compensated robot state
  // Serial.println("");
  // printArray("Compensated encoder angles: ", currentEncoderAngles, numJoints);
  // Serial.println("");

  // printArray("Desired (home) angles: ", homeJointAngles, numJoints);
  // printArray("Current estimated joint angles: ", currentJointAngles, numJoints);


  Serial.print("\n");
  Serial.println("[END SETUP]");
  Serial.print("\n");
}

void loop()
{
  asyncBlinkBuiltinLED();

  const bool debug = false;

  fastSerialRead(debug); // poll for incoming command
  parseCommand(debug); // parse command if valid one exists in buffer

  //Serial.println(readLimitSwitchState());

  //readEncoderAngles(debug);
  //sendRobotAngleUpdate();

}
