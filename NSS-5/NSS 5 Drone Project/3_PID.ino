#define ROLL_STABLE_KP 3
#define ROLL_STABLE_KI 1
#define ROLL_RATE_KP   120
#define ROLL_RATE_KI   0
#define ROLL_RATE_KD   9

#define PITCH_STABLE_KP ROLL_STABLE_KP
#define PITCH_STABLE_KI ROLL_STABLE_KI
#define PITCH_RATE_KP   ROLL_RATE_KP
#define PITCH_RATE_KI   ROLL_RATE_KI
#define PITCH_RATE_KD   ROLL_RATE_KD

#define YAW_STABLE_KP 3
#define YAW_STABLE_KI 0
#define YAW_RATE_KP 4
#define YAW_RATE_KI 0
#define YAW_RATE_KD 3

double ANGLE[3] = {0,}, RATE[3] = {0,}, lastRate[3] = {0,};
int16_t ITerm[3][2] = {0,}, rcData[6] = {0,};
bool Drive = false, Debug = false;

int16_t PID_CUT[3][2] = {
  {10000,10000},
  {10000,10000},
  {10000,10000}
};

int16_t CONTROL_CUT[3] = {
  200,
  200,
  100
};

void UpdateData()
{
  // 0 ~ 1000 -> -15 ~ 15
  if(Drive)
  {
    rcData[0] = (chData[0] - 500) >> 5; // Roll
    rcData[1] = (chData[1] - 500) >> 5; // Pitch
    int Temp = chData[3] - 500;
    if(abs(Temp) > 50) rcData[2] += Temp >> 7; // Yaw
    rcData[3] = chData[2]; // Throttle
  }
  rcData[4] = chData[4] > 500; // SW A
  rcData[5] = chData[5] > 500; // SW D
  // Angle&Gyro
  ANGLE[0] = -Angle[0];
  ANGLE[1] = -Angle[1];
  ANGLE[2] = -Angle[2];
  RATE[0] = -Rate[0];
  RATE[1] = -Rate[1];
  RATE[2] = -Rate[2];
  // Switch
  if(rcData[3] < 50) Drive = rcData[4];
}

void UpdatePID()
{
  // Update rcData
  UpdateData();
  // PID Tunes
  int16_t PTerm, DTerm, errRate, errAngle, Control[3];
  if(Drive)
  {
    errAngle = rcData[0] - ANGLE[0];
    errRate = (errAngle * ROLL_STABLE_KP) - (RATE[0] * ROLL_RATE_KP);
    PTerm = errRate;
    ITerm[0][0] += errAngle * ROLL_STABLE_KI;
    ITerm[0][1] += errRate * ROLL_RATE_KI;
    DTerm = (errRate - lastRate[0]) * ROLL_RATE_KD;
    lastRate[0] = errRate;
    Control[0] = PTerm + (ITerm[0][0] >> 7) + (ITerm[0][1] >> 7) + DTerm;
    /* PITCH */
    errAngle = rcData[1] - ANGLE[1];
    errRate = (errAngle * PITCH_STABLE_KP) - (RATE[1] * PITCH_RATE_KP);
    PTerm = errRate;
    ITerm[1][0] += errAngle * PITCH_STABLE_KI;
    ITerm[1][1] += errRate * PITCH_RATE_KI;
    DTerm = (errRate - lastRate[1]) * PITCH_RATE_KD;
    lastRate[1] = errRate;
    Control[1] = PTerm + (ITerm[1][0] >> 7) + (ITerm[1][1] >> 7) + DTerm;
    /* YAW */
    errAngle = (rcData[2] >> 3) - ANGLE[2]; // 목표값- ANGLE[2];
    errRate = (errAngle * YAW_STABLE_KP) - (RATE[2] * YAW_RATE_KP);
    PTerm = errRate;
    // ITerm[2][0] += errAngle * YAW_STABLE_KI;
    ITerm[2][1] += errRate * YAW_RATE_KI;
    DTerm = (errRate - lastRate[2]) * YAW_RATE_KD;
    lastRate[2] = errRate;
    Control[2] = PTerm + (ITerm[2][0] >> 7) + (ITerm[2][1] >> 7) + DTerm;
    /* Min&Max Cut */
    for(uint8_t i = 0; i < 3; i++)
    {
      ITerm[i][0] = constrain(ITerm[i][0], -PID_CUT[i][0], PID_CUT[i][0]);
      ITerm[i][1] = constrain(ITerm[i][1], -PID_CUT[i][1], PID_CUT[i][1]);
      Control[i] = constrain(Control[i], -CONTROL_CUT[i], CONTROL_CUT[i]);
    }
    /* Motor Write */
    QuadX(50 + rcData[3], Control[0], Control[1], Control[2]);
  }
  else WriteAll(1000);
}
