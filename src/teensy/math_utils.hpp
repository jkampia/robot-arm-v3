#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

/* returns the sign of a number: -1 if negative, 1 if positive 0 if 0 */
inline int sign(int x)
{
  return (x > 0) - (x < 0);
}

inline int sign(float x)
{
  return (x > 0.0f) - (x < 0.0f);
}



/*
Scales all values in a uint16_t vector by a scalar value
*/
inline bool scaleVector(std::vector<uint16_t>& delayVector, float scalar)
{
  if (!std::isfinite(scalar) || scalar <= 0) return false;
  for (const auto delay : delayVector)
  {
    const double scaled = std::ceil(double(delay) * scalar);
    if (scaled < 2 || scaled > UINT16_MAX) return false;
  }
  for (auto& delay : delayVector)
    delay = uint16_t(std::ceil(double(delay) * scalar));
  return true;
}



inline void arraySubtract(const float* target, const float* current, float* output, const int len)
{
  for (int i = 0; i < len; i++)
  {
    output[i] = target[i] - current[i];
  }
}


/*
normalize a vector of length 3
*/
inline bool normalizeVector(float* vector)
{
    float sum = 0.0f;
    for (int i = 0; i < 3; i++)
    {
      sum += vector[i] * vector[i];
    }
    if (sum <= 1e-12f)   // avoid divide-by-zero
    { return false; }

    float invMag = 1.0f / sqrtf(sum);

    for (int i = 0; i < 3; i++)
        vector[i] *= invMag;

    return true;
}



inline void zeroArray(int* array, const int len)
{
  for (int i = 0; i < len; i++)
  {
    array[i] = 0;
  }
}

inline void zeroArray(float* array, const int len)
{
  for (int i = 0; i < len; i++)
  {
    array[i] = 0;
  }
}



