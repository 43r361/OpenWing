#define VRX_PIN 39  // ESP32 pin GPIO39 (ADC3) connected to VRX pin
#define VRY_PIN 36  // ESP32 pin GPIO36 (ADC0) connected to VRY pin

const int SAMPLES = 10;
const int DEADZONE = 15;

const int X_MIN = 0;
const int X_CENTER = 2570;
const int X_MAX = 4095;

const int Y_MIN = 0;
const int Y_CENTER = 2630;
const int Y_MAX = 4095;

int readAverage(int pin) {
    long sum = 0;

    for (int i = 0; i < SAMPLES; i++) {
        sum += analogRead(pin);

        delayMicroseconds(500);
    }

    return sum / SAMPLES;
}

int mapJoystickDelta(int value, int center, int min, int max, int outMin, int outMax) {
    if (value < center - DEADZONE) {
        return map(value, min, center - DEADZONE, outMin, 0);
    } else if (value > center + DEADZONE) {
        return map(value, center + DEADZONE, max, 0, outMax);
    } else {
        return 0;
    }
}

void setup() {
    Serial.begin(115200);

    // Set the ADC attenuation to 11 dB (up to ~3.3V input)
    analogSetAttenuation(ADC_11db);

    pinMode(VRX_PIN, INPUT);
    pinMode(VRY_PIN, INPUT);
}

void loop() {
    int rawX = analogRead(VRX_PIN);
    int rawY = analogRead(VRY_PIN);

    int averageX = readAverage(VRX_PIN);
    int averageY = readAverage(VRY_PIN);

    int mappedX = mapJoystickDelta(averageX, X_CENTER, X_MIN, X_MAX, -2048, 2048);
    int mappedY = mapJoystickDelta(averageY, Y_CENTER, Y_MIN, Y_MAX, -2048, 2048);

    Serial.print("Raw X: ");
    Serial.print(rawX);
    Serial.print("\tRaw Y: ");
    Serial.print(rawY);
    Serial.print("\tAverage X: ");
    Serial.print(averageX);
    Serial.print("\tAverage Y: ");
    Serial.print(averageY);
    Serial.print("\tX: ");
    Serial.print(mappedX);
    Serial.print("\tY: ");
    Serial.println(mappedY);

    delay(200);
}