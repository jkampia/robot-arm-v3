// Written by Jonathan Kampia
// jonathankampia@gmail.com


#include "pin_defs.hpp"
#include "print_utils.hpp"
#include "serial_utils.hpp"
#include "math_utils.hpp"

#include <cmath>
#include <vector>


/* joint stuff */
#define numJoints 5


int   currentJointSteps[numJoints]   = {0,   0,   0,   0,   0  };
float currentJointAngles[numJoints]  = {0.0, 0.0, 0.0, 0.0, 0.0};
float currentPose[numJoints]         = {0.0, 0.0, 0.0, 0.0, 0.0};
int   enaFlags[numJoints]            = {1,   1,   1,   1,   1  };


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
void jogJoints(const float *desiredJointAngles, bool implementAcceleration = true)
{
  // calculate steps to move to satisfy angle delta
  int stepsToMove[numJoints];
  int jointSigns[numJoints];
  for (int i = 0; i < numJoints; i++) 
  {
    stepsToMove[i] = (desiredJointAngles[i] - currentJointAngles[i]) * stepsPerRad[i];
    jointSigns[i] = sign(stepsToMove[i]);
  }

  printArray("Current joint angles: ", currentJointAngles, numJoints);
  printArray("Desired joint angles: ", desiredJointAngles, numJoints);
  printArray("Steps to move: ",        stepsToMove,        numJoints);
  printArray("Joint signs:",           jointSigns,         numJoints);
  
  
  setJointDirections(jointSigns);

  // do the thing!!
  if (implementAcceleration)
  {
    goStepsAccel(stepsToMove); 
  }

  // apply movement to joint angles 
  for (int i = 0; i < numJoints; i++) 
  {
    currentJointAngles[i] += stepsToMove[i] * radsPerStep[i];
  }

  // solve forward kinematics to update end effector position
  //solve_FK(currentJointAngles, cur_pose);
}


/* 
FUNCTION PARAMS
stepsToMove: array containing # of steps each joint has to move
*/
void goStepsAccel(const int *stepsToMove) 
{
  float sumTime[numJoints] = {0.0, 0.0, 0.0, 0.0, 0.0};
  //float jointTimeScalar[numJoints] = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < numJoints; i++)
  {
    // wipe vector, maintains its space in memory
    jointDelays[i].clear(); 
    
    // populate jointDelays with new delay values
    calculateStepDelays(minStepDelay[i], abs(stepsToMove[i]), jointDelays[i]); 
    
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

  int stepCount[numJoints];
  unsigned long refTime[numJoints];
  bool jointEnableFlags[numJoints];
  for (int i = 0; i < numJoints; i++) 
  { 
    stepCount[i] = 0; 
    jointEnableFlags[i] = true;
    refTime[i] = micros();
  }
  // we are all ready to go!

  auto checkFinished = [&]() -> bool
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


  /* unfortunately this cannot be done in a loop, because the pin numbers have to be #defines or const for digitalWriteFast() :( 
     this is a blocking loop that is super timing-critical */
  while (!checkFinished()) 
  {
    // joint 0
    if (stepsToMove[0] != 0 && jointEnableFlags[0])
    {
      if (micros() - refTime[0] > jointDelays[0][stepCount[0]] * 0.5 && micros() - refTime[0] < jointDelays[0][stepCount[0]]) 
      {
        digitalWriteFast(pul0, HIGH);
      }
      else if (micros() - refTime[0] > jointDelays[0][stepCount[0]]) 
      {
        digitalWriteFast(pul0, LOW);
        stepCount[0]++; 
        refTime[0] = micros();   
      }
    }

    // joint 1
    if (stepsToMove[1] != 0 && jointEnableFlags[1])
    {
      if (micros() - refTime[1] > jointDelays[1][stepCount[1]] * 0.5 && micros() - refTime[1] < jointDelays[1][stepCount[1]]) 
      {
        digitalWriteFast(pul1, HIGH);
      }
      else if (micros() - refTime[1] > jointDelays[1][stepCount[1]]) 
      {
        digitalWriteFast(pul1, LOW);
        stepCount[1]++; 
        refTime[1] = micros();   
      }
    }

    // joint 2
    if (stepsToMove[2] != 0 && jointEnableFlags[2])
    {
      if (micros() - refTime[2] > jointDelays[2][stepCount[2]] * 0.5 && micros() - refTime[2] < jointDelays[2][stepCount[2]]) 
      {
        digitalWriteFast(pul2, HIGH);
      }
      else if (micros() - refTime[2] > jointDelays[2][stepCount[2]]) 
      {
        digitalWriteFast(pul2, LOW);
        stepCount[2]++; 
        refTime[2] = micros();   
      }
    }

    // joint 3
    if (stepsToMove[3] != 0 && jointEnableFlags[3])
    {
      if (micros() - refTime[3] > jointDelays[3][stepCount[3]] * 0.5 && micros() - refTime[3] < jointDelays[0][stepCount[3]]) 
      {
        digitalWriteFast(pul3, HIGH);
      }
      else if (micros() - refTime[3] > jointDelays[3][stepCount[3]]) 
      {
        digitalWriteFast(pul3, LOW);
        stepCount[3]++; 
        refTime[3] = micros();   
      }
    }

    // joint 4
    if (stepsToMove[4] != 0 && jointEnableFlags[4])
    {
      if (micros() - refTime[4] > jointDelays[4][stepCount[4]] * 0.5 && micros() - refTime[4] < jointDelays[4][stepCount[4]]) 
      {
        digitalWriteFast(pul4, HIGH);
      }
      else if (micros() - refTime[4] > jointDelays[4][stepCount[4]]) 
      {
        digitalWriteFast(pul4, LOW);
        stepCount[4]++; 
        refTime[4] = micros();   
      }
    }
  }

  Serial.println("Finished movement");
}



/*
Magic secret sauce function, calculates step delays required for smooth S-curve 
for a single joint

FUNCTION PARAMS
motorIndex: the joint index (0-4)
numSteps: the number of steps the joint has to move
delayVector: the joint delay vector to be populated
*/
void calculateStepDelays(const int minStepDelay, const int numSteps, std::vector<uint8_t>& delayVector) 
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
      
      float desiredAngles[numJoints];
      if (!solveIK(desiredPoseXYZPR, desiredAngles)) break;
      
      const bool implementAcceleration = true;
      jogJoints(desiredAngles, implementAcceleration);

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
