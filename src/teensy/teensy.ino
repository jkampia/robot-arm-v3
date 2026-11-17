// Written by Jonathan Kampia
// jonathankampia@gmail.com

// stepper motor wiring order: black-green-red-blue

#include "joint_info.hpp"
#include "kinematics.hpp"
#include "pin_defs.hpp"
#include "print_utils.hpp"
#include "math_utils.hpp"
#include "serial_utils.hpp"
#include "memory.hpp"

#include <cmath>
#include <array>
#include <climits>
#include <vector>

#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>

Servo gripper;
// Last commanded angle; the servo does not provide position feedback.
float currentGripperAngle = GRIPPER_ANG_MIN;
AS5600 encoder;


enum class specialMode : int
{
  NONE = 0,
  GET_ENCODER_HOME_STEPS = 1,
  BYPASS_CALIBRATION_REQ = 2,
};

//specialMode GLOBAL_MODE = specialMode::NONE;
specialMode GLOBAL_MODE = specialMode::BYPASS_CALIBRATION_REQ;
//specialMode GLOBAL_MODE = specialMode::GET_ENCODER_HOME_STEPS;

bool initial_calibration_complete = false;


/* joint stuff */
constexpr float homeJointAngles[numJoints] = {0.0, PI/2, -PI/2, PI/2, 0.0};
constexpr float returnJointAngles[numJoints] = {0.0, PI/2, -PI/2, PI/2, 0.0};

int currentJointSteps[numJoints];
float currentJointAngles[numJoints];

// object containing XYZ of each joint + origin & tooltip
kinematics::JointCoordinates currentJointPositions {};


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
      if (debug) Serial.printf("Received complete command\n");
      break;
    }
    if (inbyte == ':')
    {
      if (++bufferIndex >= serialBufferLength)
      {
        receiving = false;
        Serial.printf("Message has too many segments\n");
      }
      continue;
    }
    if (serialBuffer[bufferIndex].length() >= serialBufferNumBytes)
    {
      receiving = false;
      Serial.printf("Message segment too long\n");
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






bool checkCalibrationStatus()
{
  if (!initial_calibration_complete)
  {
    Serial.printf("Command denied: robot has not been calibrated yet!\n");
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
bool jogJoints(const float *desiredJointAngles, bool implementAcceleration = true, const int movementTimeUs = 0, const bool reportAngles = true)
{
  Serial.printf("\nNEW JOG:\n");
  printArray("Current joint angles: ", currentJointAngles, numJoints);
  printArray("Desired joint angles: ", desiredJointAngles, numJoints);

  // check validity of movement -- this needs to be fixed
  /*
  if (!checkAgainstJointLimits(desiredJointAngles))
  {
    Serial.printf("Movement invalid\n");
    return false;
  }
  */

  for (int i = 0; i < numJoints; i++)
  {
    const double steps = fabs(double(desiredJointAngles[i]) - currentJointAngles[i]) * jointInfo[i].stepsPerRad;
    if (!std::isfinite(steps) || steps > INT_MAX)
    {
      Serial.printf("Invalid joint target\n");
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
  Serial.printf("Jog completed in %.2f s\n", elapsedTimeS);

  if (reportAngles) sendRobotAngleUpdate();
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
        //Serial.printf("H\n");
      }
      else if (pulseHigh[0] && dt >= delay)
      {
        digitalWriteFast(pul0, LOW);
        pulseHigh[0] = false;
        //Serial.printf("L\n");
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
    Serial.printf("Invalid movement time provided: %d", movementTimeUs);
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

    const bool failure_conditions_if_positive = stepcount_positive && (no_joint_sign || pulse_period_invalid);

    if (stepcount_negative || failure_conditions_if_positive)
    {
      Serial.printf("Invalid step count, direction, or pulse period\n");
      return false;
    }
  }

  for (int i = 0; i < numJoints; i++)
  {
    jointDelays[i].clear();
    if (stepsToMove[i] > 0)
    {
      // Cap before storing in uint16_t so long periods cannot wrap.
      jointDelays[i].push_back(min(movementTimeUs / stepsToMove[i], max_step_delay));
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
    // Apply one common scale factor, then cap periods to storage capacity.
    // Capped joints may finish earlier than the planned segment duration.
    // Integer ceiling ensures the shortest period reaches at least the minimum.

    const float scale_factor = float(min_step_delay) / shortest_period;

    for (int i = 0; i < numJoints; i++)
    {
      for (auto& delay : jointDelays[i])
      {
        // round up to avoid dipping below the min delay
        const uint32_t scaled_delay = uint32_t(ceilf(float(delay) * scale_factor));

        delay = uint16_t(scaled_delay > uint32_t(max_step_delay) ? max_step_delay : scaled_delay);
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
      Serial.printf("Invalid step count or direction\n");
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
    //Serial.printf("%.2f\n", sumTime[i]);
  }
  //Serial.printf("Max time: %.2f\n", maxTime);

  for (int i = 0; i < numJoints; i++)
  {
    if (sumTime[i] != 0) {
      float jointTimeScalar = maxTime / sumTime[i];
      //Serial.printf("%.2f\n", jointTimeScalar);
      if (!scaleVector(jointDelays[i], jointTimeScalar))
      {
        Serial.printf("Invalid scaled acceleration pulse period\n");
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
        //Serial.printf("%d\n", rampUpStepCount);
      }
      if (i >= numSteps / 2) {
        rampUpStepCount = i;
        //Serial.printf("%d\n", rampUpStepCount);
      }
    }
    else if (i >= numSteps - rampUpStepCount) {
      n--;
      d = (d * (4 * n + 1)) / (4 * n + 1 - 2);
    }

    //Serial.printf("%u\n", static_cast<unsigned int>(uint16_t(d)));
    delayVector.push_back(uint16_t(d));
    //Serial.printf("%u\n", static_cast<unsigned int>(delay_array[i]));
  }
}





// Integrate a Cartesian trapezoidal (or triangular for short paths) speed profile.
// Each duration covers one equal-length segment, starting and ending at rest.
bool computeSegmentDelays(const double pathLengthMm, const int numSegments,
                          const double maxSpeedMmps, const double accelMmps2,
                          std::vector<int>& segmentTimes)
{
  segmentTimes.clear();
  if (!std::isfinite(pathLengthMm) || pathLengthMm <= 0 || numSegments <= 0 ||
      !std::isfinite(maxSpeedMmps) || maxSpeedMmps <= 0 ||
      !std::isfinite(accelMmps2) || accelMmps2 <= 0) return false;

  const double peakSpeed = fmin(maxSpeedMmps, sqrt(accelMmps2 * pathLengthMm));
  const double rampDistance = peakSpeed * peakSpeed / (2 * accelMmps2);
  const double rampTime = peakSpeed / accelMmps2;
  const double totalTime = 2 * rampTime + fmax(0.0, pathLengthMm - 2 * rampDistance) / peakSpeed;
  double previousTime = 0;
  segmentTimes.reserve(numSegments);
  for (int i = 0; i < numSegments; i++)
  {
    const double position = pathLengthMm * (double(i + 1) / numSegments);
    double time;
    if (position <= rampDistance)
      time = sqrt(2 * position / accelMmps2);
    else if (position >= pathLengthMm - rampDistance)
      time = totalTime - sqrt(2 * fmax(0.0, pathLengthMm - position) / accelMmps2);
    else
      time = rampTime + (position - rampDistance) / peakSpeed;

    const double durationUs = ceil((time - previousTime) * 1000000.0);
    if (!std::isfinite(durationUs) || durationUs < 1 || durationUs > INT_MAX)
    {
      segmentTimes.clear();
      return false;
    }
    segmentTimes.push_back(int(durationUs));
    previousTime = time;
  }
  return true;
}



void jogPositionJointSpace(float* desiredPoseXYZPR)
{
  float desiredAngles[numJoints];
  if (!kinematics::solveIK(desiredPoseXYZPR, desiredAngles)) return;

  const bool implementAcceleration = true;
  jogJoints(desiredAngles, implementAcceleration);
}





const float linepath_resolution_mm = 1.0;
const float linepath_speed_mmps = 50;
const float max_linepath_speed_mmps = 200;
// Cartesian acceleration for line paths; independent of jointAccel.
const float linepath_accel_mmps2 = 300;

void jogPositionLinePath(float* desiredPoseXYZPR)
{
  if (!desiredPoseXYZPR) return;
  for (int j = 0; j < numJoints; j++)
  {
    if (!std::isfinite(desiredPoseXYZPR[j]))
    {
      Serial.printf("Invalid line path target\n");
      return;
    }
  }

  float start[numJoints];
  float delta[numJoints];
  for (int axis = 0; axis < 3; axis++) start[axis] = currentJointPositions.back()[axis];
  start[3] = currentJointAngles[1] + currentJointAngles[2] + currentJointAngles[3] - kinematics::halfPi;
  start[4] = currentJointAngles[0] + currentJointAngles[4];

  arraySubtract(desiredPoseXYZPR, start, delta, numJoints);
  const float distance = sqrtf(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
  if (distance == 0 && delta[3] == 0 && delta[4] == 0) return;

  // For a pure orientation move, use the tool's angular travel as path length.
  const float path_length = distance > 0 ? distance : fmaxf(fabsf(delta[3]), fabsf(delta[4])) * (limbLengths[3] + limbLengths[4]);

  const float segments = ceilf(path_length / linepath_resolution_mm);
  // Bound planning memory on the microcontroller.
  if (!std::isfinite(segments) || segments > 4096 || segments < 1)
  {
    Serial.printf("Line path exceeds planning capacity\n");
    return;
  }

  const int count = int(segments);

  // Equal-length segments share one duration; goStepsNoAccel handles joint scaling.
  const double duration = ceil(1000000.0 * path_length / count / linepath_speed_mmps);
  if (!std::isfinite(duration) || duration < 1 || duration > INT_MAX) return;
  const int segment_time_us = int(duration);

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
      Serial.printf("Line path stopped: segment rejected\n");
      return;
    }
  }

}


void jogPositionLinePathAccel(float* desiredPoseXYZPR)
{
  if (!desiredPoseXYZPR) return;
  for (int j = 0; j < numJoints; j++)
  {
    if (!std::isfinite(desiredPoseXYZPR[j]))
    {
      Serial.printf("Invalid line path target\n");
      return;
    }
  }

  float start[numJoints];
  float delta[numJoints];
  for (int axis = 0; axis < 3; axis++) start[axis] = currentJointPositions.back()[axis];
  start[3] = currentJointAngles[1] + currentJointAngles[2] + currentJointAngles[3] - kinematics::halfPi;
  start[4] = currentJointAngles[0] + currentJointAngles[4];

  arraySubtract(desiredPoseXYZPR, start, delta, numJoints);
  const float distance = sqrtf(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
  if (distance == 0 && delta[3] == 0 && delta[4] == 0) return;

  // For a pure orientation move, use the tool's angular travel as path length.
  const float path_length = distance > 0 ? distance : fmaxf(fabsf(delta[3]), fabsf(delta[4])) * (limbLengths[3] + limbLengths[4]);

  const float segments = ceilf(path_length / linepath_resolution_mm);
  // Bound planning memory on the microcontroller.
  if (!std::isfinite(segments) || segments > 4096 || segments < 1)
  {
    Serial.printf("Line path exceeds planning capacity\n");
    return;
  }

  const int count = int(segments);

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

  std::vector<int> segmentTimes;
  if (!computeSegmentDelays(path_length, count, max_linepath_speed_mmps,
                            linepath_accel_mmps2, segmentTimes))
  {
    Serial.printf("Invalid line path timing or segment time exceeds capacity\n");
    return;
  }

  for (int i = 0; i < count; i++)
  {
    if (!jogJoints(trajectory[i].data(), false, segmentTimes[i]))
    {
      Serial.printf("Line path stopped: segment rejected\n");
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
  if (!std::isfinite(angle))
  {
    Serial.printf("Invalid gripper angle\n");
    return;
  }

  // calculate pwm setpoint
  float angle_clamped = clampGripperAngle(angle);
  float pwm_setpt = mapGripperAngle(angle_clamped);

  gripper.writeMicroseconds((int)pwm_setpt);
  currentGripperAngle = angle_clamped;

  Serial.printf("Wrote gripper angle: %.2f -> pwm setpoint: %.2fus\n", angle_clamped, pwm_setpt);
}




// Blocking move from the last commanded angle, in degrees.
void setGripperAngleSlow(const float angle)
{
  static_assert(GRIPPER_RATE_DEGPS > 0, "Gripper rate must be positive");
  static_assert(GRIPPER_RESOLUTION_DEG > 0, "Gripper resolution must be positive");
  if (!std::isfinite(angle))
  {
    Serial.printf("Invalid gripper angle\n");
    return;
  }

  const float target = clampGripperAngle(angle);
  while (currentGripperAngle != target)
  {
    const float remaining = target - currentGripperAngle;
    const float step = fminf(fabsf(remaining), GRIPPER_RESOLUTION_DEG);
    const float nextAngle = fabsf(remaining) <= GRIPPER_RESOLUTION_DEG
                            ? target : currentGripperAngle + (remaining > 0 ? step : -step);

    // Scale the last interval to its actual step so partial steps keep the rate.
    const double intervalUs = double(step) * 1000000.0 / GRIPPER_RATE_DEGPS;
    const unsigned long intervalMs = static_cast<unsigned long>(intervalUs / 1000.0);
    delay(intervalMs);
    delayMicroseconds(static_cast<unsigned int>(intervalUs - double(intervalMs) * 1000.0));
    setGripperAngle(nextAngle);
  }
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
        Serial.printf("WARNING: AS5600 not detected on TCA9548A channel %u\n", static_cast<unsigned int>(encoderMuxChannels[i]));
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
bool readEncoderAngles(const bool debug = false, const int jointIndex = -1)
{
  uint32_t startTimeUs = micros();
  bool allReadsOk = true;

  for (int i = jointIndex < 0 ? 0 : jointIndex; i < (jointIndex < 0 ? numJoints : jointIndex + 1); i++)
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
        currentEncoderAngles[i] = temp * 2*PI / 4096.0;
      }
    }

    if (!readOk)
    {
      allReadsOk = false;
      /*
      if (debug)
      {
        Serial.printf("Failed to read AS5600 on TCA9548A channel %u\n", static_cast<unsigned int>(encoderMuxChannels[i]));
      }
      */
    }
  }
  uint32_t elapsedTimeUs = micros() - startTimeUs;

  if (debug)
  {
    printArray("Encoder steps: ", currentEncoderSteps, numJoints);
    Serial.printf("Read encoder angles in %lu us\n", static_cast<unsigned long>(elapsedTimeUs));
  }

  return allReadsOk;
}


void sendRobotAngleUpdate()
{
  Serial.printf("<%d", int(serialCommand::JOINT_ANGLE_UPDATE));

  for (int i = 0; i < numJoints; i++)
  {
    Serial.printf(":%.4f", currentJointAngles[i]);
  }

  for (int i = 0; i < numJoints; i++)
  {
    Serial.printf(":%d", currentJointSteps[i]);
  }

  Serial.printf(">\n");
}


// Reasons must not contain frame delimiters (<, >) or field separators (:).
void sendMoveFailed(const serialCommand command, const char* reason)
{
  Serial.printf("<%d:%d:%s>\n", int(serialCommand::MOVE_FAILED), int(command), reason);
}



/*
Acts on serialBuffer global array of commands
*/
void parseCommand(const bool debug = false)
{
  if (!newCommand) return;

  // HOME is zero: never let toInt() turn an empty/malformed ID into HOME.
  const String command = serialBuffer[0];
  bool validCommand = command.length() > 0;
  for (unsigned int i = 0; i < command.length(); i++)
  {
    if (command[i] < '0' || command[i] > '9') validCommand = false;
  }
  // Bound conversion to avoid integer overflow on oversized command IDs.
  if (!validCommand || command.length() > 9)
  {
    Serial.printf("Invalid command ID\n");
    newCommand = false;
    return;
  }
  const serialCommand commandSignature = static_cast<serialCommand>(command.toInt());

  // if calibration has not been completed, and we are not in the bypass mode, return
  // also allow through a few choice command signatures
  if (!checkCalibrationStatus() 
        && commandSignature != serialCommand::HOME
        && commandSignature != serialCommand::REQUEST_ENCODER_STEPS_UPDATE
        && commandSignature != serialCommand::OVERRIDE_CALIBRATION_SAFETY)
  {
    newCommand = false;
    return;
  }


  Serial.printf("Received %s command\n", serialCommandToString(commandSignature).c_str());

  // only send confirmation if the command is supported
  if (commandSupported(commandSignature))
  {
    sendReceiveConfirmation(commandSignature);
  }
  
  switch(commandSignature)
  {
    case serialCommand::REQUEST_ENCODER_STEPS_UPDATE:
    {
      // Report encoder steps in J0-J4 order; failed reads contribute zero.
      int encoderSteps[numJoints] = {0};
      for (int i = 0; i < numJoints; i++)
      {
        if (readEncoderAngles(false, i)) encoderSteps[i] = currentEncoderSteps[i];
      }
      Serial.printf("<%d", int(serialCommand::ENCODER_STEPS_UPDATE));
      for (int i = 0; i < numJoints; i++)
      {
        Serial.printf(":%d", encoderSteps[i]);
      }
      Serial.printf(">\n");
      break;
    }

    case serialCommand::HOME:
    {
      initial_calibration_complete = false;
      setBuiltinLEDRate(5);
      const int j_idx = 4;
      if (!homeWithEncoderFeedback(j_idx))
      {
        sendMoveFailed(commandSignature, "J4 encoder homing failed");
        break;
      }

      // Keep intermediate angle updates from marking HOME complete in the backend.
      float desiredAngles[numJoints];
      memcpy(desiredAngles, currentJointAngles, sizeof(desiredAngles));
      desiredAngles[j_idx] += PI/2;
      if (!jogJoints(desiredAngles, true, 0, false))
      {
        sendMoveFailed(commandSignature, "J4 clearance movement rejected");
        break;
      }

      if (!homeWithLimitSwitchFeedback()) break; // Helper reports the HOME failure.

      memcpy(desiredAngles, currentJointAngles, sizeof(desiredAngles));
      desiredAngles[j_idx] = homeJointAngles[j_idx];
      if (!jogJoints(desiredAngles, true, 0, false))
      {
        sendMoveFailed(commandSignature, "J4 restore movement rejected");
        break;
      }

      initial_calibration_complete = true;
      setBuiltinLEDRate(1);
      Serial.printf("Done homing\n");
      sendRobotAngleUpdate();
      break;
    }

    case serialCommand::HOME_J4:
    {
      if (!homeWithEncoderFeedback(4))
      {
        initial_calibration_complete = false;
        setBuiltinLEDRate(5);
        sendMoveFailed(commandSignature, "J4 encoder homing failed");
        break;
      }
      // Homing one joint cannot establish calibration for the entire arm.
      sendRobotAngleUpdate();
      break;
    }

    case serialCommand::RETURN:
    {
      returnToHome();
      break;
    }

    case serialCommand::OVERRIDE_CALIBRATION_SAFETY:
    {
      initial_calibration_complete = true;
      break;
    }

    case serialCommand::DISABLE_ALL:
    case serialCommand::ENABLE_ALL:
    {
      const int flag = commandSignature == serialCommand::DISABLE_ALL ? 1 : 0;
      for (int i = 0; i < numJoints; i++)
      {
        enaFlags[i] = flag;
      }
      parseEnaFlags();
      break;
    }

    case serialCommand::DISABLE_JOINT:
    case serialCommand::ENABLE_JOINT:
    {
      // Joint indices are zero-based; require an unsigned integer argument.
      const String jointArg = serialBuffer[1];
      bool validIndex = jointArg.length() > 0;
      for (unsigned int i = 0; i < jointArg.length(); i++)
      {
        if (jointArg[i] < '0' || jointArg[i] > '9') validIndex = false;
      }
      const long j_idx = jointArg.toInt();
      if (!validIndex || j_idx < 0 || j_idx >= numJoints)
      {
        sendMoveFailed(commandSignature, "Invalid joint index");
        break;
      }

      enaFlags[j_idx] = commandSignature == serialCommand::DISABLE_JOINT ? 1 : 0;
      parseEnaFlags();
      break;
    }

    case serialCommand::TGT_ANGLES_JOINT_SPACE:
    {
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
          Serial.printf("Position command incomplete: arg%d missing\n", i);
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

    case serialCommand::TGT_POSITION_LINE_PATH:
    {
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
          Serial.printf("Position command incomplete: arg%d missing\n", i);
          commandValid = false;
          break;
        }
      }

      if (!commandValid) break;

      jogPositionLinePath(desiredPoseXYZPR);

      break;
    }

    case serialCommand::TGT_POSITION_LINE_PATH_ACCEL:
    {
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
          Serial.printf("Position command incomplete: arg%d missing\n", i);
          commandValid = false;
          break;
        }
      }

      if (!commandValid) break;

      jogPositionLinePathAccel(desiredPoseXYZPR);

      break;
    }

    case serialCommand::GRIPPER_SET_ANGLE:
    {
      float angle = serialBuffer[1].toFloat();
      setGripperAngle(angle);

      break;
    }

    case serialCommand::GRIPPER_CLOSE:
    {
      float close_angle = GRIPPER_ANG_CLOSE;
      setGripperAngleSlow(close_angle);

      break;
    }

    case serialCommand::GRIPPER_OPEN:
    {
      float open_angle = GRIPPER_ANG_OPEN;
      setGripperAngleSlow(open_angle);

      break;
    }

    default:
    {
      Serial.printf("Received unknown command signature\n");
      break;
    }
  }

  newCommand = false;
  Serial.printf("\n");
}



// send confirmation to frontend that a given command was received
void sendReceiveConfirmation(serialCommand confirmCommand)
{
  Serial.printf("<%d:%d>\n", int(serialCommand::CONFIRM_RECEIVE), int(confirmCommand));
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
  Home the selected joint
*/
bool homeWithEncoderFeedback(const int j_idx)
{
  if (j_idx < 0 || j_idx >= numJoints || !encoderAvailable[j_idx])
  {
    Serial.printf("Encoder homing unavailable for joint %d\n", j_idx);
    return false;
  }
  if (enaFlags[j_idx] != 0)
  {
    Serial.printf("Encoder homing rejected: joint %d is disabled\n", j_idx);
    return false;
  }

  const int tolerance = 1;
  const uint32_t timeoutMs = 30000;
  const uint32_t startMs = millis();
  int stepsToMove[numJoints] = {0};
  int jointSigns[numJoints] = {0};
  stepsToMove[j_idx] = 1;

  // Read only this encoder; missing encoders on other joints are irrelevant.
  if (!readEncoderAngles(false, j_idx)) return false;
  int error = encoderAngleHomingStepTargets[j_idx] - currentEncoderSteps[j_idx];
  while (abs(error) > tolerance)
  {
    if (uint32_t(millis() - startMs) >= timeoutMs)
    {
      Serial.printf("Encoder homing timeout for joint %d\n", j_idx);
      return false;
    }
    const int prevError = error;
    jointSigns[j_idx] = sign(error);
    if (!goStepsNoAccel(stepsToMove, jointSigns, 400)) return false;
    if (!readEncoderAngles(false, j_idx)) return false;
    error = encoderAngleHomingStepTargets[j_idx] - currentEncoderSteps[j_idx];
    if (abs(error) > tolerance && sign(error) != sign(prevError))
    {
      Serial.printf("Encoder homing crossed target outside tolerance\n");
      return false;
    }
  }

  currentJointAngles[j_idx] = homeJointAngles[j_idx];
  currentJointSteps[j_idx] = homeJointAngles[j_idx] * jointInfo[j_idx].stepsPerRad;
  currentJointPositions = kinematics::solveFK(currentJointAngles);

  if (j_idx == 2)
  {
    float desiredAngles[numJoints];
    memcpy(desiredAngles, currentJointAngles, sizeof(desiredAngles));
    desiredAngles[2] -= PI/2;
    if (!jogJoints(desiredAngles, true, 0, false)) return false;
  }
  return true;
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
    Serial.printf("Limit input: matching samples=%lu/%lu, match percent=%.2f, "
                  "required percent=%lu, expected=%s\n",
                  static_cast<unsigned long>(matchingSamples),
                  static_cast<unsigned long>(totalSamples),
                  100.0f * matchingSamples / totalSamples,
                  static_cast<unsigned long>(limitSwitchMinMatchPercent),
                  expectedState ? "HIGH (open)" : "LOW (closed)");
  }
  return valid;
}



const uint32_t limitSwitchSeekTimeoutMs = 30000;


// Home joints 0, 1, 2, 3 with limit switch feedback
bool homeWithLimitSwitchFeedback()
{
  initial_calibration_complete = false;

  setBuiltinLEDRate(5);

  int stepsToMove[numJoints] = {0};
  int jointSigns[numJoints] = {0};
  int stepsMoved[numJoints] = {0};

  bool allJointsHomed = true;



  for (int i = numJoints-2; i >= 0; i--)
  {
    const int movementTimeUs = homingStepDelay[i];

    if (!homingFlags[i])
    {
      Serial.printf("Skipping joint %d\n", i);

      allJointsHomed = false;

      continue;
    }

    if (enaFlags[i] != 0)
    {
      sendMoveFailed(serialCommand::HOME, "Homing joint is disabled");
      return false;
    }

    if (stepsToMoveAfterLimitSwitchClick[i] <= 0 || abs(limitSwitchSeekSigns[i]) != 1)
    {
      Serial.printf("Homing aborted: invalid joint configuration\n");
      sendMoveFailed(serialCommand::HOME, "Invalid homing joint configuration");
      return false;
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
        Serial.printf("Homing aborted: seek limit reached for joint %d\n", i);
        sendMoveFailed(serialCommand::HOME, "Homing seek timeout");
        return false;
      }
      if (readLimitSwitchState())
      {
        // Pause on majority HIGH. A confirmed hit ends seeking; a confirmed LOW
        // means it was a glitch, so retry without resetting the seek timeout.
        if (waitForStableLimitSwitchState(true, false)) break;
        if (!waitForStableLimitSwitchState(false))
        {
          Serial.printf("Homing aborted: ambiguous limit input for joint %d\n", i);
          sendMoveFailed(serialCommand::HOME, "Ambiguous homing limit input");
          return false;
        }
        continue;
      }

      if (!goStepsNoAccel(stepsToMove, jointSigns, movementTimeUs))
      {
        sendMoveFailed(serialCommand::HOME, "Homing seek movement rejected");
        return false;
      }
      seekSteps++;

      // update step count
      stepsMoved[i] += stepsToMove[i];
    }

    // reverse away from the switch to the calibrated home offset
    stepsToMove[i] = stepsToMoveAfterLimitSwitchClick[i];
    jointSigns[i] = -limitSwitchSeekSigns[i];
    if (!goStepsAccel(stepsToMove, jointSigns))
    {
      sendMoveFailed(serialCommand::HOME, "Homing backoff movement rejected");
      return false;
    }

    if (!waitForStableLimitSwitchState(false))
    {
      Serial.printf("Homing aborted: limit circuit did not settle closed after backoff\n");
      sendMoveFailed(serialCommand::HOME, "Limit circuit did not settle closed after backoff");
      return false;
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
    Serial.printf("Partial homing complete; calibration remains incomplete\n");
    sendMoveFailed(serialCommand::HOME, "Partial homing; calibration incomplete");
    return false;
  }

  return true;
}

/*
  return to home position (different from intial homing)
*/
void returnToHome()
{
  float desiredAngles[numJoints];
  memcpy(desiredAngles, returnJointAngles, numJoints * sizeof(float));

  const bool implementAcceleration = true;
  if (!jogJoints(desiredAngles, implementAcceleration))
  {
    Serial.printf("Return to home failed: movement rejected\n");
    sendMoveFailed(serialCommand::RETURN, "Return movement rejected");
    return;
  }

  Serial.printf("Returned to home position\n");
}





void setup()
{
  Serial.begin(921600);

  Serial.printf("\n[SETUP]\n\n");

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
  const float fast_led_rate = 10; // hz
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

  printArray("Current joint steps: ", currentJointSteps, numJoints);
  printArray("Current joint angles: ", currentJointAngles, numJoints);
  printEndEffectorPosition(currentJointPositions);

  // post vector allocation memory
  memory::printFreeMemory();

  // read encoder angles & assign joint angles
  // readEncoderAngles();

  // debug print compensated robot state
  // Serial.printf("\n");
  // printArray("Compensated encoder angles: ", currentEncoderAngles, numJoints);
  // Serial.printf("\n");

  // printArray("Desired (home) angles: ", homeJointAngles, numJoints);
  // printArray("Current estimated joint angles: ", currentJointAngles, numJoints);

  if (GLOBAL_MODE == specialMode::BYPASS_CALIBRATION_REQ)
  {
    initial_calibration_complete = true;
    Serial.printf("Calibration bypass mode!!!");
  }


  Serial.printf("\n[END SETUP]\n\n");
}

void loop()
{
  asyncBlinkBuiltinLED();

  const bool debug = false;

  fastSerialRead(debug); // poll for incoming command
  parseCommand(debug); // parse command if valid one exists in buffer

  //Serial.printf("%d\n", static_cast<int>(readLimitSwitchState()));

  //const bool encoder_debug = true;
  //readEncoderAngles(encoder_debug);
  //sendRobotAngleUpdate();

}
