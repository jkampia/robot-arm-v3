#pragma once

#include <array>
#include <cmath>

// DH parameters
#include "joint_info.hpp"

constexpr float limbLengths[numJoints] = {142.22, 265.0, 200, 0.0, 38.0};

namespace kinematics
{
using Matrix4 = std::array<std::array<float, 4>, 4>;
using Coordinates = std::array<float, 3>;
// Base origin, five DH joint positions, and the tool tip, in millimeters.
using JointCoordinates = std::array<Coordinates, numJoints + 2>;
constexpr float halfPi = 1.57079632679489661923f;

// Fixed DH geometry; theta is supplied by the current joint angles.
constexpr float dhAlpha[numJoints] = {halfPi, 0.0f, 0.0f, halfPi, 0.0f};
constexpr float dhD[numJoints] = {limbLengths[0], 0.0f, 0.0f, 0.0f, limbLengths[3]};
constexpr float dhA[numJoints] = {0.0f, limbLengths[1], limbLengths[2], 0.0f, 0.0f};

inline Matrix4 multiplyMatrices(const Matrix4& left, const Matrix4& right)
{
  Matrix4 result{};
  for (int row = 0; row < 4; row++)
    for (int col = 0; col < 4; col++)
      for (int k = 0; k < 4; k++)
        result[row][col] += left[row][k] * right[k][col];
  return result;
}

inline Coordinates extractCoordinates(const Matrix4& transform)
{
  return {{transform[0][3], transform[1][3], transform[2][3]}};
}

// Standard Denavit-Hartenberg transform, matching ARM_5DOF.tfMatrix.
inline Matrix4 tfMatrix(float theta, float alpha, float a, float d)
{
  const float ct = std::cos(theta), st = std::sin(theta);
  const float ca = std::cos(alpha), sa = std::sin(alpha);
  return {{{{ct, -st * ca, st * sa, a * ct}},
           {{st, ct * ca, -ct * sa, a * st}},
           {{0.0f, sa, ca, d}},
           {{0.0f, 0.0f, 0.0f, 1.0f}}}};
}

inline Coordinates toolDirectionFromJointAngles(const float* jointAngles)
{
  const float pitch = jointAngles[1] + jointAngles[2] + jointAngles[3] - halfPi;
  return {{std::cos(pitch) * std::cos(jointAngles[0]),
           std::cos(pitch) * std::sin(jointAngles[0]), std::sin(pitch)}};
}

// Angles in radians; lengths and returned coordinates in millimeters.
inline JointCoordinates solveFK(const float* jointAngles)
{
  JointCoordinates coordinates{};
  Matrix4 product = tfMatrix(jointAngles[0], dhAlpha[0], dhA[0], dhD[0]);
  coordinates[1] = extractCoordinates(product);
  for (int i = 1; i < numJoints; i++)
  {
    product = multiplyMatrices(product, tfMatrix(jointAngles[i], dhAlpha[i], dhA[i], dhD[i]));
    coordinates[i + 1] = extractCoordinates(product);
  }
  const Coordinates direction = toolDirectionFromJointAngles(jointAngles);
  for (int axis = 0; axis < 3; axis++)
    coordinates[numJoints + 1][axis] = coordinates[numJoints][axis] + direction[axis] * limbLengths[4];
  return coordinates;
}



inline bool checkIKSolution(const float* angles)
{
  for (int i = 0; i < numJoints; i++)
  {
    if (!std::isfinite(angles[i]))
    {
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
inline bool solveIK(const float* targetPose, float* targetJointAngles)
{
  if (!checkIKSolution(targetPose)) return false;

  // unpack inputs for clarity
  float x = targetPose[0];
  float y = targetPose[1];
  float z = targetPose[2];
  float theta4 = targetPose[3];
  float theta5 = targetPose[4];

  float l1 = limbLengths[0];
  float l2 = limbLengths[1];
  float l3 = limbLengths[2];
  float l4 = limbLengths[3];
  float l5 = limbLengths[4];

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
  // Allow only floating-point roundoff at the workspace boundary.
  if (cos_phi < -1.000001f || cos_phi > 1.000001f || rsquared <= 0) return false;
  cos_phi = fmaxf(-1.0f, fminf(1.0f, cos_phi));
  float theta3 = acos(cos_phi);

  // shoulder
  float alpha = atan2(z4, hypot(x4, y4));
  float beta = atan2(l3 * sin(theta3), l2 + l3 * cos(theta3));
  float theta2 = alpha + beta;

  // flip elbow angle to match arm orientation
  // AFTER it is used to calculate theta2
  theta3 *= -1;

  // apply offsets to theta4 as it is provided in world frame
  // NOT last link frame, and FK expects last link frame
  float j4_offset = theta2 + theta3;
  theta4 -= j4_offset - halfPi;


  // assign to output array
  targetJointAngles[0] = theta1;
  targetJointAngles[1] = theta2;
  targetJointAngles[2] = theta3;
  targetJointAngles[3] = theta4;
  targetJointAngles[4] = theta5;

  return checkIKSolution(targetJointAngles);
}



} // namespace kinematics
