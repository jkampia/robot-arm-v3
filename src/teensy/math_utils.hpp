#pragma once

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
Scales all values in a uint8_t vector by a scalar value
*/
inline void scaleVector(std::vector<uint8_t>& delayVector, float scalar) 
{
  uint64_t scaledTotalTime = 0;
  for (auto& delay : delayVector) 
  {
    delay *= scalar; 
    scaledTotalTime += delay;
  }
  //Serial.println("Scaled time: " + String(scaled_time));
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


