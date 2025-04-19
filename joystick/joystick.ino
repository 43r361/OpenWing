#define VRX1_PIN (35)
#define VRY1_PIN (34)
#define VRX2_PIN (39)
#define VRY2_PIN (36)

const int SAMPLES = 10;
const int DEADZONE = 15;

const int X_MIN = 0;
const int X_CENTER = 1846;
const int X_MAX = 4095;

const int Y_MIN = 0;
const int Y_CENTER = 1875;
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

    pinMode(VRX1_PIN, INPUT);
    pinMode(VRY1_PIN, INPUT);
    pinMode(VRX2_PIN, INPUT);
    pinMode(VRY2_PIN, INPUT);
}

void loop() {
    int rawX1 = analogRead(VRX1_PIN);
    int rawY1 = analogRead(VRY1_PIN);

    int averageX1 = readAverage(VRX1_PIN);
    int averageY1 = readAverage(VRY1_PIN);

    int mappedX1 = mapJoystickDelta(averageX1, X_CENTER, X_MIN, X_MAX, -25, 25);
    int mappedY1 = mapJoystickDelta(averageY1, Y_CENTER, Y_MIN, Y_MAX, -25, 25);

    int rawX2 = analogRead(VRX2_PIN);
    int rawY2 = analogRead(VRY2_PIN);

    int averageX2 = readAverage(VRX2_PIN);
    int averageY2 = readAverage(VRY2_PIN);

    int mappedX2 = mapJoystickDelta(averageX2, X_CENTER, X_MIN, X_MAX, -25, 25);
    int mappedY2 = mapJoystickDelta(averageY2, Y_CENTER, Y_MIN, Y_MAX, -25, 25);

    Serial.print("Raw X 1: ");
    Serial.print(rawX1);
    Serial.print("\tRaw Y 1: ");
    Serial.print(rawY1);

    Serial.print("\tAverage X 1: ");
    Serial.print(averageX1);
    Serial.print("\tAverage Y 1: ");
    Serial.print(averageY1);

    Serial.print("\tMapped X 1: ");
    Serial.print(mappedX1);
    Serial.print("\tMapped Y 1: ");
    Serial.println(mappedY1);

    Serial.print("Raw X 2: ");
    Serial.print(rawX2);
    Serial.print("\tRaw Y 2: ");
    Serial.print(rawY2);

    Serial.print("\tAverage X 2: ");
    Serial.print(averageX2);
    Serial.print("\tAverage Y 2: ");
    Serial.print(averageY2);

    Serial.print("\tMapped X 2: ");
    Serial.print(mappedX2);
    Serial.print("\tMapped Y 2: ");
    Serial.println(mappedY2);

    delay(200);
}