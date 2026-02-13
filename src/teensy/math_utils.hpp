#pragma once

/* returns the sign of a number: -1 if negative, 1 if positive 0 if 0 */
inline int sign(int x)
{
  return (x > 0) - (x < 0);
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