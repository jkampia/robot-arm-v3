/* print helpers */

inline void printArray(const String& msg, const float* array, int len)
{
  Serial.print(msg);
  Serial.print("[ ");
  for (int i = 0; i < len-1; i++) 
  {
    Serial.print(array[i]);
    Serial.print(", ");
  }
  Serial.print(array[len-1]);
  Serial.print(" ]\n");
}

inline void printArray(const String& msg, const int * array, int len)
{
  Serial.print(msg);
  Serial.print("[ ");
  for (int i = 0; i < len-1; i++) 
  {
    Serial.print(array[i]);
    Serial.print(", ");
  }
  Serial.print(array[len-1]);
  Serial.print(" ]\n");
}

inline void printArray(const String& msg, const bool * array, int len)
{
  Serial.print(msg);
  Serial.print("[ ");
  for (int i = 0; i < len-1; i++) 
  {
    Serial.print(array[i]);
    Serial.print(", ");
  }
  Serial.print(array[len-1]);
  Serial.print(" ]\n");
}

