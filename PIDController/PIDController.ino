#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <ESP32Servo.h>  // ESP32Servo library installed by Library Manager

//sensor macros

#define I2C_SDA 21
#define I2C_SCL 22
#define RGB_PIN 48

#define LSB_PER_DEG_PER_SEC 65.5
#define LSB_PER_G 4096
#define CALIBRATION_CYCLES 2000


#define ESC_PIN1 (9)
#define ESC_PIN2 (20)
#define ESC_PIN3 (18)  // unuse
#define ESC_PIN4 (10)

// in Hz
#define FREQUENCY (250)
// in s
#define PERIOD (1.0 / (float)FREQUENCY)

// throtle in ppm values
#define MIN_SPEED 1148  // 1148 | 1000 both values depend on the motor
#define MAX_SPEED 1832  // 1832


// sensor vals
float rateRoll = 0, ratePitch = 0, rateYaw = 0;
float angleRoll = 0, anglePitch = 0, angleYaw = 0;
// standard deviation on the initial angle: 2 deg
float kalmanAngleRoll = 0, kalmanUncertaintyAngleRoll = 2 * 2;
float kalmanAnglePitch = 0, kalmanUncertaintyAnglePitch = 2 * 2;

// index 0 is the angle prediction and index 1 is the uncertainty
float kalman1dOutput[] = { 0, 0 };

Servo motors[4];

uint32_t loopTimer = 0;

// in a cross or H motor configuration
// motor 1 front right
// motor 2 bottom right
// motor 3 bottom left
// motor 4 front left

// this type of controller expects the values to be in degrees per second
// range -70 - 70 degrees for yaw, roll and pitch
// range for throttle is 0 to 1
typedef struct
{
  float roll;
  float yaw;
  float pitch;
  float throttle;
} struct_message;

typedef struct
{
  float P;
  float I;
  float D;
} pid_constants_t;

typedef struct
{
  float Pitch;
  float Roll;
  float Yaw;
} angle_val_t;

typedef struct
{
  float Iterm;
  float Error;
  float PIDOutput;
} pid_return_t;

pid_constants_t rateRollPID = { 0.6, 3.5, 0.03 };
pid_constants_t ratePitchPID = { 0.6, 3.5, 0.03 };
pid_constants_t rateYawPID = { 2, 12, 0 };

angle_val_t ErrorRate = { 0, 0, 0 };
angle_val_t PreviousErrorRate = { 0, 0, 0 };
angle_val_t PreviousItermRate = { 0, 0, 0 };

struct_message controllerVals = { 0, 0, 0, 0 };

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&controllerVals, incomingData, sizeof(controllerVals));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Roll: ");
  Serial.println(controllerVals.roll);
  Serial.print("Pitch: ");
  Serial.println(controllerVals.pitch);
  Serial.print("Yaw: ");
  Serial.println(controllerVals.yaw);
  Serial.print("Throttle: ");
  Serial.print(controllerVals.throttle);
  Serial.println();
}

pid_return_t pid_equation(float Error, pid_constants_t PID_const, float PreviousError, float PreviousIterm) {
  float Pterm = PID_const.P * Error;
  float Iterm = PreviousIterm + PID_const.I * (Error + PreviousError) * PERIOD;

  // use to avoid integral wind up
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;

  float Dterm = PID_const.D * (Error - PreviousError) / PERIOD;
  float PIDOutput = Pterm + Iterm + Dterm;
  // limit to -400  400
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  pid_return_t output;
  output.Iterm = Iterm;
  output.Error = Error;
  output.PIDOutput = PIDOutput;
  return output;
}

void reset_pid() {
  PreviousErrorRate.Pitch = 0;
  PreviousErrorRate.Roll = 0;
  PreviousErrorRate.Yaw = 0;
  PreviousItermRate.Pitch = 0;
  PreviousItermRate.Roll = 0;
  PreviousItermRate.Yaw = 0;
}

void motor_setup() {
  pinMode(ESC_PIN1, OUTPUT);
  pinMode(ESC_PIN2, OUTPUT);
  pinMode(ESC_PIN3, OUTPUT);
  pinMode(ESC_PIN4, OUTPUT);
  motors[0].attach(ESC_PIN1);
  motors[1].attach(ESC_PIN2);
  motors[2].attach(ESC_PIN3);
  motors[3].attach(ESC_PIN4);

  delay(3000);
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(1400);
  }
  delay(1000);
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(MIN_SPEED);
  }
  delay(3500);
  Serial.println("Available for input!");
}

void setup() {
  Serial.begin(115200);
  rgbLedWrite(RGB_PIN, RGB_BRIGHTNESS, 0, 0);  // Red

  WiFi.mode(WIFI_MODE_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  sensor_setup();

  motor_setup();

  // avoid accidental lift
  while (controllerVals.throttle > 0.10) {
    delay(4);
  }
  // indicate that setup is done
  rgbLedWrite(RGB_PIN, 0, RGB_BRIGHTNESS, 0);  // Green
}

float throttle_to_input(float throttle) {
  return (MAX_SPEED - MIN_SPEED) * throttle + MIN_SPEED;
}

float input_to_throttle(float input) {
  return (input - MIN_SPEED) / (MAX_SPEED - MIN_SPEED);
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

  // printKalmanAngles();

  ErrorRate.Roll = controllerVals.roll - rateRoll;
  ErrorRate.Pitch = controllerVals.pitch - ratePitch;
  ErrorRate.Yaw = controllerVals.yaw - rateYaw;

  pid_return_t rollReturn = pid_equation(ErrorRate.Roll, rateRollPID, PreviousErrorRate.Roll, PreviousItermRate.Roll);
  pid_return_t pitchReturn = pid_equation(ErrorRate.Pitch, ratePitchPID, PreviousErrorRate.Pitch, PreviousItermRate.Pitch);
  pid_return_t yawReturn = pid_equation(ErrorRate.Yaw, rateYawPID, PreviousErrorRate.Yaw, PreviousItermRate.Yaw);
  // update error
  PreviousErrorRate.Roll = rollReturn.Error;
  PreviousErrorRate.Pitch = pitchReturn.Error;
  PreviousErrorRate.Yaw = yawReturn.Error;
  // update Iterm
  PreviousItermRate.Roll = rollReturn.Iterm;
  PreviousItermRate.Pitch = pitchReturn.Iterm;
  PreviousItermRate.Yaw = yawReturn.Iterm;

  float InputPitch = pitchReturn.PIDOutput;  // in direct input
  float InputRoll = rollReturn.PIDOutput;
  float InputYaw = yawReturn.PIDOutput;


  // limit throttle
  controllerVals.throttle = min(0.8f, controllerVals.throttle);
  float InputThrottle = throttle_to_input(controllerVals.throttle);

  float input[4];
  // 0.00 - 0.25;
  // This is for a H or Cross motor position

  input[0] = InputThrottle - InputPitch - InputRoll - InputYaw;  // to show basic control
  input[1] = InputThrottle + InputPitch - InputRoll + InputYaw;
  input[2] = InputThrottle + InputPitch + InputRoll - InputYaw;
  input[3] = InputThrottle - InputPitch + InputRoll + InputYaw;

  for (int i = 0; i < 4; i++) {
    // limit so the motors don't stop
    input[i] = max(input[i], throttle_to_input(0.10f));  // 0.10 should be 0.20 used for showcasing
    // stop it from exeding max limit
    input[i] = min(input[i], throttle_to_input(0.60f));  // should be 1.00 but 0.60 is used for showcasing
  }


  // stop the motor if the values are low
  if (InputThrottle < 0.04) {
    for (int i = 0; i < 4; i++) {
      input[i] = MIN_SPEED;
    }
    reset_pid();
  }

  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(input[i]);
  }

  // do nothing until it's time for the next loop (every [PERIOD] seconds)
  while (micros() - loopTimer < PERIOD * 1000000)
    ;
  loopTimer = micros();
}
