#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>
#include <Wire.h>

#include "I2Cdev.h"

Adafruit_BMP085 bmp;
HMC5883L_Simple compass;

#define I2C_SDA 21
#define I2C_SCL 22

#define LSB_PER_DEG_PER_SEC 65.5
#define LSB_PER_G 4096
#define CALIBRATION_CYCLES 2000
// in Hz
#define FREQUENCY (250)
// in s
#define PERIOD (1.0 / (float)FREQUENCY)

float rateRoll = 0, ratePitch = 0, rateYaw = 0;
float rateCalibrationRoll = 0, rateCalibrationPitch = 0, rateCalibrationYaw = 0;
bool calibrated = false;

float accX = 0, accY = 0, accZ = 0;
// these values can be set to fix offsets in the accelerometer readings
float accCalibrationX = 0, accCalibrationY = 0, accCalibrationZ = 0;
float angleRoll = 0, anglePitch = 0, angleYaw = 0;

uint32_t loopTimer;

// standard deviation on the initial angle: 2 deg
float kalmanAngleRoll = 0, kalmanUncertaintyAngleRoll = 2 * 2;
float kalmanAnglePitch = 0, kalmanUncertaintyAnglePitch = 2 * 2;

// index 0 is the angle prediction and index 1 is the uncertainty
float kalman1dOutput[] = {0, 0};

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
    float gain = uncertainty / (uncertainty + 3 * 3);

    // update the predicted state
    state += gain * (measurement - state);

    // update the uncertainty
    uncertainty *= 1 - gain;

    // set the output
    kalman1dOutput[0] = state;
    kalman1dOutput[1] = uncertainty;
}

void printRates() {
    // Serial.println("Rates:");
    Serial.print(rateRoll);
    Serial.print(", ");
    Serial.print(ratePitch);
    Serial.print(", ");
    Serial.println(rateYaw);
}

void printAcceleration() {
    // Serial.println("Acceleration:");
    Serial.print(accX);
    Serial.print(", ");
    Serial.print(accY);
    Serial.print(", ");
    Serial.println(accZ);
}

void printAngles() {
    // Serial.println("Angles:");
    Serial.print(angleRoll);
    Serial.print(", ");
    Serial.print(anglePitch);
    Serial.print(", ");
    Serial.println(angleYaw);
}

void printKalmanAngles() {
    // Serial.println("Kalman angles:");
    Serial.print(kalmanAngleRoll);
    Serial.print(", ");
    Serial.print(-kalmanAnglePitch);
    Serial.print(", ");
    Serial.println(angleYaw);
}

void readAccelerometer() {
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

void bootGy87() {
    // start the mpu6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // configure the accelerometer's full scale range (4096 LSB/g)
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

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

void setup() {
    Serial.begin(115200);

    // set the clock speed to 400 kHz
    Wire.begin(I2C_SDA, I2C_SCL, 400000);

    // give the mpu6050 time to start
    delay(250);

    bootGy87();

    // initialize bmp085
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {
        }
    }

    // initialize hmc5883l
    compass.SetDeclination(23, 35, 'E');
    compass.SetSamplingMode(COMPASS_SINGLE);
    compass.SetScale(COMPASS_SCALE_130);
    compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}

void loop() {
    readAll();

    // calculate the roll and pitch angles
    kalman1d(kalmanAngleRoll, kalmanUncertaintyAngleRoll, rateRoll, angleRoll);
    kalmanAngleRoll = kalman1dOutput[0];
    kalmanUncertaintyAngleRoll = kalman1dOutput[1];

    kalman1d(kalmanAnglePitch, kalmanUncertaintyAnglePitch, ratePitch, anglePitch);
    kalmanAnglePitch = kalman1dOutput[0];
    kalmanUncertaintyAnglePitch = kalman1dOutput[1];

    angleYaw += rateYaw * PERIOD;

    if (angleYaw >= 360) {
        angleYaw -= 360;
    }

    printKalmanAngles();

    // do nothing until it's time for the next loop (every [PERIOD] seconds)
    while (micros() - loopTimer < PERIOD * 1000000);
    loopTimer = micros();
}
