float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;


void gyro_signals(void) {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  RateRoll=(float)gx/65.5;
  RatePitch=(float)gy/65.5;
  RateYaw=(float)gz/65.5;
  AccX=(float)ax/4096;
  AccY=(float)ay/4096;
  AccZ=(float)az/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}