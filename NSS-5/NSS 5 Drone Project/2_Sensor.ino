#include <Wire.h>

double Angle[3] = {0}, Rate[3] = {0}, DT = 0;
int Offset[7] = {
  9900, // Acc X
  8220,  // Acc Y
  12500, // Acc Z
  0,     // Temp
  707,   // Gyro X
  -710,  // Gyro Y
  810};  // Gyro Z
uint32_t imuTime = 0;

void InitIMU()
{
  uint8_t i;
  Wire.begin();
  Wire.setClock(400000);
  // Register 107 - Power Management 1
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(0);
  Wire.endTransmission();
  // Register 26 - Configuration
  for(i = 2; i <= 7; i++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(26);
    Wire.write((i << 3) | 0x04);
    Wire.endTransmission();
  }
  // Register 27 – Gyroscope Configuration
  Wire.beginTransmission(0x68);
  Wire.write(27);
  Wire.write(0b11111000);
  Wire.endTransmission();
  // Register 28 – Accelerometer Configuration
  Wire.beginTransmission(0x68);
  Wire.write(28);
  Wire.write(0b11100000);
  Wire.endTransmission();
}

void UpdateIMU()
{
  uint8_t i;
  int16_t rawVector;
  static int16_t RAW[7] = {0}, Reverse = 1;
  int32_t currentMicros = micros();
  double rawAngle[2];
  // 2.5ms
  while(micros() - imuTime < 2500); currentMicros = micros();
  // Calc DT
  DT = (currentMicros - imuTime) * 0.000001F;
  imuTime = currentMicros;
  // Calc Angle&Gyro
  Wire.beginTransmission(0x68);
  Wire.write(59);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  for(i = 0; i < 7; i++) RAW[i] = (Wire.read() << 8 | Wire.read()) - Offset[i];
  for(i = 0; i < 3; i++) Rate[i] = RAW[i+4] / 16.4 * DT;
  // 360 Degree
  if(RAW[2] > 0)
  {
    Reverse = 1;
    rawVector = sqrt(pow(RAW[0], 2) + pow(RAW[2], 2));
    rawAngle[0] = atan2(RAW[1], rawVector) * RAD_TO_DEG;
    rawVector = sqrt(pow(RAW[1], 2) + pow(RAW[2], 2));
    rawAngle[1] = -atan2(RAW[0], rawVector) * RAD_TO_DEG;
    for(i = 0; i < 2; i++) Angle[i] = (Angle[i] + Rate[i]) * 0.98 + rawAngle[i] * 0.02;
  }
  else
  {
    // 뒤집어진 경우 각가속도값을 사용
    for(i = 0; i < 2; i++)
    {
      Angle[i] += Rate[i] * Reverse;
      if(abs(Angle[i]) >= 180) Reverse *= -1;
    }
  }
  Angle[2] += Rate[2];
}
