Adafruit_BMP085 bmp;
HMC5883L_Simple Compass;

float rateCalibrationRoll = 0, rateCalibrationPitch = 0, rateCalibrationYaw = 0;
bool calibrated = false;

float accX = 0, accY = 0, accZ = 0;
// TODO: calibrate
float accCalibrationX = 0, accCalibrationY = 0, accCalibrationZ = 0;

// state - angle calculated with kalman
// input - rotation rate
// measurement - accelerometer angle
// why constants?
void kalman1d(float state, float uncertainty, float input, float measurement) {
    // predict the current state of the system
    state += PERIOD * input;

    // calculate the uncertainty
    uncertainty += PERIOD * PERIOD * 4 * 4;

    // calculate the kalman gain based on previous calculations
    float gain = uncertainty * 1 / (1 * uncertainty + 3 * 3);

    // update the predicted state
    state += gain * (measurement - state);

    // update the uncertainty
    uncertainty *= 1 - gain;

    // set the output
    kalman1dOutput[0] = state;
    kalman1dOutput[1] = uncertainty;
}

void printRates() {
    Serial.println("Rates:");
    Serial.print(rateRoll);
    Serial.print(", ");
    Serial.print(ratePitch);
    Serial.print(", ");
    Serial.println(rateYaw);
}

void printAcceleration() {
    Serial.println("Acceleration:");
    Serial.print(accX);
    Serial.print(", ");
    Serial.print(accY);
    Serial.print(", ");
    Serial.println(accZ);
}

void printAngles() {
    Serial.println("Angles:");
    Serial.print(angleRoll);
    Serial.print(", ");
    Serial.println(anglePitch);
}

void printKalmanAngles() {
    // Serial.println("Kalman angles:");
    Serial.print(-kalmanAngleRoll);
    Serial.print(", ");
    Serial.print(kalmanAnglePitch);
    Serial.print(", ");
    Serial.println(angleYaw);
}

void readAccelerometer() {
    // configure the accelerometer's full scale range (4096 LSB/g)
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // access the registers, storing the accelerometer values
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();

    // read register 0x3B-0x40 (59-64), containing measurements
    Wire.requestFrom(0x68, 6);

    // the data for x, y and z is contained in 2 registers each
    // that's why we need two reads for each value (and a shift)
    int16_t accXLSB = Wire.read() << 8 | Wire.read();
    int16_t accYLSB = Wire.read() << 8 | Wire.read();
    int16_t accZLSB = Wire.read() << 8 | Wire.read();

    // convert the measurements from LSB to g
    accX = (float)accXLSB / LSB_PER_G + accCalibrationX;
    accY = (float)accYLSB / LSB_PER_G + accCalibrationY;
    accZ = (float)accZLSB / LSB_PER_G + accCalibrationZ;

    // calculate angles
    angleRoll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    // idk if we should have a "-" in front (seems wrong)
    anglePitch = -atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
}

void readGyro() {
    // activate the gyroscope's low pass filter (reduces vibration noise)
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    // configure the gyroscope's full scale range (65.5 LSB/deg/s)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();

    // access the registers, storing the gyroscope values
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();

    // read registers 0x43-0x48 (67-72), containing measurements
    Wire.requestFrom(0x68, 6);

    // the data for x, y and z is contained in 2 registers each
    // that's why we need two reads for each value (and a shift)
    int16_t rateXLSB = Wire.read() << 8 | Wire.read();
    int16_t rateYLSB = Wire.read() << 8 | Wire.read();
    int16_t rateZLSB = Wire.read() << 8 | Wire.read();

    // convert the measurements from LSB to deg/s
    rateRoll = (float)rateXLSB / LSB_PER_DEG_PER_SEC;
    ratePitch = (float)rateYLSB / LSB_PER_DEG_PER_SEC;
    rateYaw = (float)rateZLSB / LSB_PER_DEG_PER_SEC;

    if (calibrated) {
        rateRoll -= rateCalibrationRoll;
        ratePitch -= rateCalibrationPitch;
        rateYaw -= rateCalibrationYaw;
    }
}

void readAll() {
    readAccelerometer();
    readGyro();
}

void bootMpu6050(void) {
    // start the mpu6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // calibrate the gyroscope
    Serial.println("Start calibration");

    for (int i = 0; i < CALIBRATION_CYCLES; i++) {
        readGyro();

        rateCalibrationRoll += rateRoll;
        rateCalibrationPitch += ratePitch;
        rateCalibrationYaw += rateYaw;

        delay(1);
    }

    rateCalibrationRoll /= CALIBRATION_CYCLES;
    rateCalibrationPitch /= CALIBRATION_CYCLES;
    rateCalibrationYaw /= CALIBRATION_CYCLES;

    calibrated = true;

    Serial.println("Calibration finished\nValues:");
    Serial.print(rateCalibrationRoll);
    Serial.print(", ");
    Serial.print(rateCalibrationPitch);
    Serial.print(", ");
    Serial.println(rateCalibrationYaw);
    Serial.println("End");
}

void sensor_setup()
{
  // set the clock speed to 400 kHz
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  // give the mpu6050 time to start
  delay(250);
  bootMpu6050();

  // initialize bmp085
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {
    }
  }

  // initialize hmc5883l
  Compass.SetDeclination(23, 35, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}
