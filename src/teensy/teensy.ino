// Written by Jonathan Kampia
// jonathankampia@gmail.com

// stepper motor wiring order: red-blue-green-black


#include "pin_defs.hpp"
#include "print_utils.hpp"
#include "serial_utils.hpp"
#include "math_utils.hpp"

#include <cmath>
#include <vector>


/* joint stuff */
constexpr int numJoints = 5; 

constexpr int homeJointSteps[numJoints]    = {0,   0,   0,   0,   0  };
constexpr float homeJointAngles[numJoints] = {0.0, 0.0, 0.0, 0.0, 0.0};
constexpr float homePose[numJoints]        = {0.0, 0.0, 0.0, 0.0, 0.0};

int   currentJointSteps[numJoints];
float currentJointAngles[numJoints];
float currentPose[numJoints];

int   enaFlags[numJoints] = {1, 1, 1, 1, 1};

const float jointLengths[numJoints]  = {100,  300,  250,  50,   50};
const float gearReduction[numJoints] = {10.0, 10.0, 20.0, 5.0,  1.0};
const int stepsPerRev[numJoints]     = {3200, 3200, 3200, 3200, 3200};
const int minStepDelay[numJoints]    = {100,  100,  100,  100,  100};

float stepsPerRad[numJoints];
float radsPerStep[numJoints];
float stepsPerDeg[numJoints];
float degsPerStep[numJoints];

std::vector<uint8_t> jointDelays[numJoints];


/* serial */
String serialBuffer[serialBufferLength];
bool newCommand = false;


void setup() 
{
  Serial.begin(921600);

  Serial.print("\n");
  Serial.println("< NEW PROGRAM >");
  Serial.print("\n");

  for (int i = 0; i < serialBufferLength; i++)
  {
    serialBuffer[i].reserve(serialBufferNumBytes);
  }

  for (int i = 0; i < numJoints; i++)
  {
    jointDelays[i].reserve(32000);

    currentJointSteps[i] = homeJointSteps[i];
    currentJointAngles[i] = homeJointAngles[i];
    currentPose[i] = homePose[i];
  }

  printArray("Joint lengths: ", jointLengths, numJoints);
  calculateJointInfo();
  printFreeMemory();


  Serial.print("\n");
  Serial.println("< END SETUP >");
  Serial.print("\n");
}

void loop() 
{
  fastSerialRead(true); // poll for incoming command
  parseCommand(); // parse command if valid one exists in buffer
}



/*
Reads messages in the format: <val1:val2:val3>
assigns to global string array serialBuffer
*/
void fastSerialRead(bool debug)
{
  uint64_t startTime = micros();

  int bufferIndex = 0;
  int totalBytesRead = 0; 

  /* blocking based on length of msg */
  while (Serial.available() > 0) 
  {
    char inbyte = Serial.read();
     
    if (inbyte == '<' && totalBytesRead == 0) 
    {
      /* wipe buffer on receiving valid start character */
      for (int i = 0; i < serialBufferLength; i++)
      {
        serialBuffer[i] = ""; 
      }
    }
    else if (inbyte == '>' && totalBytesRead != 0) 
    {
      /* only read one <msg> per loop */
      totalBytesRead++; 
      newCommand = true;
      if (debug)
      {
        uint64_t stopTime = micros(); 
        int elapsedTime = stopTime - startTime;
        Serial.println("Read " + String(totalBytesRead) + " bytes in " + String(elapsedTime) + " us");
      } 
      break;
    }
    else if (inbyte == ':') 
    {
      bufferIndex++;
      if (bufferIndex > serialBufferLength)
      {
        Serial.println("Message exceeds " + String(serialBufferLength) + " segments");
        break;
      }
    }
    else 
    {
      serialBuffer[bufferIndex] += inbyte;   
      if (serialBuffer[bufferIndex].length() > serialBufferNumBytes)
      {
        Serial.println("Message segment exceeds " + String(serialBufferNumBytes) + " bytes");
        break;
      }
    }

    totalBytesRead++; 
  }
}


/*
Calculates some constants based on robot arm parameters
*/
void calculateJointInfo()
{
  for (int i = 0; i < numJoints; i++)
  {
    int actualStepsPerRev = stepsPerRev[i] * gearReduction[i];
    
    const float revsPerRad = 1/(2*PI);

    stepsPerRad[i] = actualStepsPerRev * revsPerRad; 
    radsPerStep[i] = 1 / stepsPerRad[i];

    const float revsPerDeg = 1/360.0;

    stepsPerDeg[i] = actualStepsPerRev * revsPerDeg; 
    degsPerStep[i] = 1 / revsPerDeg;
  }

  printArray("Steps per rad: ", stepsPerRad, numJoints);
}



/*
Set directions for all joints
Currently using digitalWriteFast so each dir pin has to be a compile time static obj

FUNCTION PARAMS
jointSigns: direction of joint rotation (+1 for CCW, -1 for CW)
*/
void setJointDirections(const int *jointSigns)
{
  if (jointSigns[0] < 0) digitalWriteFast(dir0, LOW);
  else digitalWriteFast(dir0, HIGH);
  if (jointSigns[1] < 0) digitalWriteFast(dir1, LOW);
  else digitalWriteFast(dir1, HIGH);
  if (jointSigns[2] < 0) digitalWriteFast(dir2, LOW);
  else digitalWriteFast(dir2, HIGH);
  if (jointSigns[3] < 0) digitalWriteFast(dir3, LOW);
  else digitalWriteFast(dir3, HIGH);
  if (jointSigns[4] < 0) digitalWriteFast(dir4, LOW);
  else digitalWriteFast(dir4, HIGH);
}




/* 
Supposed to be a wrapper that handles joint-space movements with and without acceleration
Right now it kind of only does S-curve accel movements

FUNCTION PARAMS
desiredJointAngles: array containing each joint's desired angle
implementAcceleration: whether or not to use acceleration S-curve or static speed, default true
*/
void jogJoints(const float *desiredJointAngles, bool implementAcceleration = true, const int movementTimeUs = 0)
{
  // calculate steps to move to satisfy angle delta
  int stepsToMove[numJoints];
  int jointSigns[numJoints];
  for (int i = 0; i < numJoints; i++) 
  {
    float diff = desiredJointAngles[i] - currentJointAngles[i];
    stepsToMove[i] = abs(diff) * stepsPerRad[i];
    jointSigns[i] = sign(diff);
  }

  printArray("Current joint angles: ", currentJointAngles, numJoints);
  printArray("Desired joint angles: ", desiredJointAngles, numJoints);
  printArray("Steps to move: ",        stepsToMove,        numJoints);
  printArray("Joint signs: ",           jointSigns,         numJoints);

  uint32_t startTimeUs = micros();

  // do the thing!!
  if (implementAcceleration)
  {
    goStepsAccel(stepsToMove, jointSigns); 
  }
  else
  {
    goStepsNoAccel(stepsToMove, jointSigns, movementTimeUs);
  }

  uint32_t stopTimeUs = micros();
  float elapsedTimeS = float(stopTimeUs - startTimeUs) / 1000000;

  printArray("Final joint angles: ", currentJointAngles, numJoints);
  printArray("Final joint steps: ",  currentJointSteps,  numJoints);
  Serial.println("Jog completed in " + String(elapsedTimeS) + " s");

  // solve forward kinematics to update end effector position
  //solve_FK(currentJointAngles, cur_pose);
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
  unsigned long refTime[numJoints];
  bool jointEnableFlags[numJoints];
  for (int i = 0; i < numJoints; i++) 
  { 
    stepCount[i] = 0; 
    jointEnableFlags[i] = true;
    refTime[i] = micros();
  }

  while (!checkFinished(stepCount, stepsToMove, jointEnableFlags)) 
  {
    // joint 0
    if (stepsToMove[0] != 0 && jointEnableFlags[0])
    {
      auto delay = jointDelays[0].front();
      if (implementAcceleration) { delay = jointDelays[0][stepCount[0]]; }
      uint32_t now = micros();
      uint32_t dt = now - refTime[0];
      uint32_t half = delay >> 1;

      if (dt > half && dt < delay) 
      {
        digitalWriteFast(pul0, HIGH);
      }
      else if (dt >= delay)
      {
        digitalWriteFast(pul0, LOW);
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

      if (dt > half && dt < delay) 
      {
        digitalWriteFast(pul1, HIGH);
      }
      else if (dt >= delay)
      {
        digitalWriteFast(pul1, LOW);
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

      if (dt > half && dt < delay) 
      {
        digitalWriteFast(pul2, HIGH);
      }
      else if (dt >= delay)
      {
        digitalWriteFast(pul2, LOW);
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

      if (dt > half && dt < delay) 
      {
        digitalWriteFast(pul3, HIGH);
      }
      else if (dt >= delay)
      {
        digitalWriteFast(pul3, LOW);
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

      if (dt > half && dt < delay) 
      {
        digitalWriteFast(pul4, HIGH);
      }
      else if (dt >= delay)
      {
        digitalWriteFast(pul4, LOW);
        stepCount[4]++; 
        refTime[4] = now;
      }
    }
  }

  // add # of steps moved to current joint step position
  // apply movement to joint angles based on actual steps taken
  // not perceived angular movement
  for (int i = 0; i < numJoints; i++)
  {
    currentJointSteps[i] += stepCount[i] * jointSigns[i];
    currentJointAngles[i] = currentJointSteps[i] * radsPerStep[i];
  }
}



void goStepsNoAccel(const int* stepsToMove, const int* jointSigns, const int movementTimeUs)
{
  if (movementTimeUs <= 0) 
  { 
    Serial.print("Invalid movement time provided: " + String(movementTimeUs));
    return; 
  }

  int tempArray[numJoints];
  for (int i = 0; i < numJoints; i++)
  {
    jointDelays[i].clear();
    jointDelays[i].front() = movementTimeUs / stepsToMove[i];
    tempArray[i] = jointDelays[i].front();
  }

  printArray("Joint step delays: ", tempArray, numJoints);

  const bool implementAcceleration = false;
  doRawSteps(stepsToMove, jointSigns, implementAcceleration);
}


/* 
FUNCTION PARAMS
stepsToMove: array containing # of steps each joint has to move
*/
void goStepsAccel(const int *stepsToMove, const int* jointSigns) 
{
  float sumTime[numJoints] = {0.0, 0.0, 0.0, 0.0, 0.0};
  //float jointTimeScalar[numJoints] = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < numJoints; i++)
  {
    // wipe vector, maintains its space in memory
    jointDelays[i].clear(); 
    // populate jointDelays with new delay values
    calculateStepDelaysAccel(minStepDelay[i], abs(stepsToMove[i]), jointDelays[i]); 
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
      scaleVector(jointDelays[i], jointTimeScalar);
    }
  }

  const bool implementAcceleration = false;
  doRawSteps(stepsToMove, jointSigns, implementAcceleration);
}



/*
Magic secret sauce function, calculates step delays required for smooth S-curve 
for a single joint

FUNCTION PARAMS
motorIndex: the joint index (0-4)
numSteps: the number of steps the joint has to move
delayVector: the joint delay vector to be populated
*/
void calculateStepDelaysAccel(const int minStepDelay, const int numSteps, std::vector<uint8_t>& delayVector) 
{ 
  const float angle = 1;
  const float accel = 0.1;
  /* some magic number idk why it's this */
  const float delay0 = 2000 * sqrt(2 * angle / accel) * 0.67703;

  float d = delay0;
  int n = 0; 
  int rampUpStepCount = 0;
  //int totalSteps = 0;

  for (int i = 0; i < numSteps; i++) {
    if (rampUpStepCount == 0) {
      n++; 
      d = d - (2 * d) / (4 * n + 1);
      if (d <= minStepDelay) {
        d = minStepDelay; 
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
    delayVector.push_back(d);
    //Serial.println(delay_array[i]);  
  }
}



bool checkIKSolution(const float* angles)
{
  for (int i = 0; i < numJoints; i++)
  {
    if (isnan(angles[i]))
    {
      printArray("Bad angle array: ", angles, numJoints);
      Serial.println("IK solution invalid: nan angle");
      return false;
    }
  }

  return true;
}


/*
geometric IK solve

FUNCTION PARAMS
targetPose: array of length [numJoints] containing x,y,z,pitch,roll of end effector
pitch & roll given in radians
*/
bool solveIK(const float* targetPose, float* targetJointAngles)
{
  uint64_t startTime = micros();

  // unpack inputs for clarity
  float x = targetPose[0];
  float y = targetPose[1];
  float z = targetPose[2];
  float theta4 = targetPose[3];
  float theta5 = targetPose[4];

  float l1 = jointLengths[0];
  float l2 = jointLengths[1];
  float l3 = jointLengths[2];
  float l4 = jointLengths[3];
  float l5 = jointLengths[4];

  // convert to radians
  //theta4 = m.radians(theta4)
  //theta5 = m.radians(theta5)

  // base yaw
  float theta1 = atan2(y, x);

  // effective wrist offset
  float wrist_len = l4 + l5;
  float horz = wrist_len * cos(theta4);
  float x4 = x - horz * cos(theta1);
  float y4 = y - horz * sin(theta1);
  float z4 = z - wrist_len * sin(theta4) - l1;

  // distance from shoulder to wrist
  float rsquared = x4*x4 + y4*y4 + z4*z4;

  // elbow
  float cos_phi = (rsquared - l2*l2 - l3*l3) / (2.0 * l2 * l3);
  float theta3 = acos(cos_phi);

  // shoulder
  float alpha = asin(z4 / sqrt(rsquared));
  float beta = atan(l3 * sin(theta3) / (l2 + l3 * cos(theta3)));
  float theta2 = alpha + beta;

  // flip elbow angle to match arm orientation
  // AFTER it is used to calculate theta2
  theta3 *= -1; 

  // apply offsets to theta4 as it is provided in world frame
  // NOT last link frame, and FK expects last link frame
  float j4_offset = theta2 + theta3;
  theta4 -= j4_offset - PI/2;


  // assign to output array
  targetJointAngles[0] = theta1;
  targetJointAngles[1] = theta2;
  targetJointAngles[2] = theta3;
  targetJointAngles[3] = theta4;
  targetJointAngles[4] = theta5;

  uint64_t stopTime = micros(); 
  int timePassedUs = stopTime - startTime;
  Serial.println("Solved IK in " + String(timePassedUs) + " us");

  return checkIKSolution(targetJointAngles);
}



void jogPositionJointSpace(float* desiredPoseXYZPR)
{
  float desiredAngles[numJoints];
  if (!solveIK(desiredPoseXYZPR, desiredAngles)) return;
  
  const bool implementAcceleration = true;
  jogJoints(desiredAngles, implementAcceleration);
}



void jogPositionLinePath(float* desiredPoseXYZPR)
{
  float deltaVector[3]; // contains only x, y, z
  arraySubtract(desiredPoseXYZPR, currentPose, deltaVector, 3);
  
  
}



/*
Acts on serialBuffer global array of commands
*/
void parseCommand()
{
  if (!newCommand) return;
  
  serialCommand commandSignature =
    static_cast<serialCommand>(serialBuffer[0].toInt());
  //Serial.println(serialBuffer[0].toInt());
  
  switch(commandSignature)
  {
    case serialCommand::HOME:
    {
      Serial.println("Received home command");  
      break;
    }

    case serialCommand::TGT_ANGLES_JOINT_SPACE:
    {
      Serial.println("Received target angle joint space command");

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
      const int movementTimeUs = 1000000;
      jogJoints(desiredAngles, implementAcceleration, movementTimeUs); 
      break;
    }

    case serialCommand::TGT_ANGLES_JOINT_SPACE_ACCEL:
    {
      Serial.println("Received target angle joint space accel command");

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
      Serial.println("Received target position joint space accel command");

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

    default:
    {
      Serial.println("Received unknown command signature");
      break;
    }
  }
  
  newCommand = false;
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
