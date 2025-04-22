#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager

#define ESC_PIN1 (19)
#define ESC_PIN2 (20)
#define ESC_PIN3 (18) 
#define ESC_PIN4 (21)

#define MIN_SPEED 1148
#define MAX_SPEED 1832

Servo motors[4];

int val = MIN_SPEED; // throtle value from serial input

void setup() {
  Serial.begin(115200);
  pinMode(ESC_PIN1, OUTPUT);
  pinMode(ESC_PIN2, OUTPUT);
  pinMode(ESC_PIN3, OUTPUT);
  pinMode(ESC_PIN4, OUTPUT);

  motors[0].attach(ESC_PIN1);
  motors[1].attach(ESC_PIN2);
  motors[2].attach(ESC_PIN3);
  motors[3].attach(ESC_PIN4);

  // add a longer delay if necessary
  delay(1000);
  for (int i=0;i<4;i++)
  {
    // sending arm signal
    motors[i].writeMicroseconds(1400);
  }
  
  delay(1000);
  
  for (int i=0;i<4;i++)
  {
    motors[i].writeMicroseconds(MIN_SPEED);
  }
  delay(3500);
  Serial.println("Available for input!");
}

void loop() {
  if (Serial.available()) {
    int tmp = Serial.parseInt();
    if (tmp != 0) val = tmp;
  }
  // limit val
  val = max(val, MIN_SPEED);
  val = min(val, MAX_SPEED);

  for (int i=0;i<4;i++)
  {
    motors[i].writeMicroseconds(val);
  }
  delay(10); // Wait for a bit
}