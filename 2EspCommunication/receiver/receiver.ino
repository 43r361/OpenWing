#include <esp_now.h>
#include <WiFi.h>

#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager

#define ESC_PIN1 (9)
#define ESC_PIN2 (20)
#define ESC_PIN3 (18) // unuse
#define ESC_PIN4 (10)

#define MIN_SPEED 1148 // 1148 | 1000 both values depend on the motor
#define MAX_SPEED 1832 // 1832

Servo motors[4];

// in a cross or H motor configuration
// motor 1 front right
// motor 2 bottom right
// motor 3 bottom left
// motor 4 front left

typedef struct
{
  float roll;
  float yaw;
  float pitch;
  float throttle;
} struct_message;

struct_message myData = {0, 0, 0, 0};

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Roll: ");
  Serial.println(myData.roll);
  Serial.print("Pitch: ");
  Serial.println(myData.pitch);
  Serial.print("Yaw: ");
  Serial.println(myData.yaw);
  Serial.print("Throttle: ");
  Serial.print(myData.throttle);
  Serial.println();
}

void motor_setup()
{
  pinMode(ESC_PIN1, OUTPUT);
  pinMode(ESC_PIN2, OUTPUT);
  pinMode(ESC_PIN3, OUTPUT);
  pinMode(ESC_PIN4, OUTPUT);
  motors[0].attach(ESC_PIN1);
  motors[1].attach(ESC_PIN2);
  motors[2].attach(ESC_PIN3);
  motors[3].attach(ESC_PIN4);

  delay(3000);
  for (int i = 0; i < 4; i++)
  {
    motors[i].writeMicroseconds(1400);
  }
  delay(1000);
  for (int i = 0; i < 4; i++)
  {
    motors[i].writeMicroseconds(MIN_SPEED);
  }
  delay(3500);
  Serial.println("Available for input!");
}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  motor_setup();
}

void loop()
{
  float inputPerc[4];
  int input[4];
  myData.throttle = min(0.8f, myData.throttle);

  // 0.00 - 0.25;

  // This is for a H or Cross motor position
  inputPerc[0] = myData.throttle - myData.pitch - myData.roll - myData.yaw; // to show basic control
  inputPerc[1] = myData.throttle + myData.pitch - myData.roll + myData.yaw;
  inputPerc[2] = myData.throttle + myData.pitch + myData.roll - myData.yaw;
  inputPerc[3] = myData.throttle - myData.pitch + myData.roll + myData.yaw;
  for (int i = 0; i < 4; i++)
  {
    inputPerc[i] = max(inputPerc[i], 0.10f); // 0.10 should be 0.20 used for showcasing
    inputPerc[i] = min(inputPerc[i], 0.60f); // should be 1.00 but 0.60 is used for showcasing
    input[i] = inputPerc[i] * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
  }

  for (int i = 0; i < 4; i++)
  {
    motors[i].writeMicroseconds(input[i]);
  }

  delay(1000);
}
