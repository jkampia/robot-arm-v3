#pragma once

/* print helpers */

inline void printArray(const String& msg, const float* array, int len)
{
  Serial.printf("%s[ ", msg.c_str());
  for (int i = 0; i < len-1; i++)
  {
    Serial.printf("%.2f, ", array[i]);
  }
  Serial.printf("%.2f ]\n", array[len-1]);
}

inline void printArray(const String& msg, const int * array, int len)
{
  Serial.printf("%s[ ", msg.c_str());
  for (int i = 0; i < len-1; i++)
  {
    Serial.printf("%d, ", array[i]);
  }
  Serial.printf("%d ]\n", array[len-1]);
}

inline void printArray(const String& msg, const uint16_t * array, int len)
{
  Serial.printf("%s[ ", msg.c_str());
  for (int i = 0; i < len-1; i++)
  {
    Serial.printf("%u, ", static_cast<unsigned int>(array[i]));
  }
  Serial.printf("%u ]\n", static_cast<unsigned int>(array[len-1]));
}

inline void printArray(const String& msg, const bool * array, int len)
{
  Serial.printf("%s[ ", msg.c_str());
  for (int i = 0; i < len-1; i++) 
  {
    Serial.printf("%d, ", static_cast<int>(array[i]));
  }
  Serial.printf("%d ]\n", static_cast<int>(array[len-1]));
}
