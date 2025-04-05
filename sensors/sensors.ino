#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>
#include "DShotRMT.h"

MPU6050 accelgyro;
Adafruit_BMP085 bmp;
HMC5883L_Simple Compass;


int16_t ax, ay, az;
int16_t gx, gy, gz;

// #define RGB_BUILTIN 48

#define I2C_SDA 35
#define I2C_SCL 36

bool blinkState = false;
// DSh

void setup() {


  Serial.begin(115200);
  Serial.println("Wire begin!");

  Wire.begin(I2C_SDA, I2C_SCL, 100000);

  Serial.println("Wire end!");

  // initialize devices
  Serial.println("Initializing I2C devices...");

  // initialize bmp085
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  // initialize mpu6050
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true);  // set bypass mode for gateway to hmc5883L
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

  // initialize hmc5883l
  Compass.SetDeclination(23, 35, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}

void loop() {
  // Serial.println("Hallo");
  // print_angles();
  gyro_signals();
  print_acc();
  delay(1000);
}
