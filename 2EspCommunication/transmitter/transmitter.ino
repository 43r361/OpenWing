#include <WiFi.h>
#include <esp_now.h>

#define VRX1_PIN (35)
#define VRY1_PIN (34)
#define VRX2_PIN (39)
#define VRY2_PIN (36)

// in Hz
#define FREQUENCY (250)
// in s
#define PERIOD (1 / (float)FREQUENCY)

// how many times we read before taking the average result
const int SAMPLES = 10;
// add a small deadzone to mitigate stick drift from the center
const int DEADZONE = 5;

// joystick 1 constants (center gets calibrated in setup())
const int X1_MIN = 0;
int X1_CENTER = 1846;
const int X1_MAX = 4095;

const int Y1_MIN = 0;
int Y1_CENTER = 1875;
const int Y1_MAX = 4095;

// joystick 1 constants (center gets calibrated in setup())
const int X2_MIN = 0;
int X2_CENTER = 1846;
const int X2_MAX = 4095;

const int Y2_MIN = 0;
int Y2_CENTER = 1875;
const int Y2_MAX = 4095;

// the time when loop() executed last
uint32_t loopTimer;

// read input [SAMPLES] times and take the average
int readAverage(int pin) {
    long sum = 0;

    for (int i = 0; i < SAMPLES; i++) {
        sum += analogRead(pin);

        delayMicroseconds(500);
    }

    return sum / SAMPLES;
}

// map the read value while taking the center and deadzone variables into account
int mapJoystickDelta(int value, int center, int min, int max, int outMin, int outCenter, int outMax) {
    if (value < center - DEADZONE) {
        return map(value, min, center - DEADZONE, outMin, 0);
    } else if (value > center + DEADZONE) {
        return map(value, center + DEADZONE, max, 0, outMax);
    } else {
        return outCenter;
    }
}

// the MAC address where we try to send the processed values
// 84:cc:a8:47:10:fc (current esp)
// 30:30:f9:69:55:20 (old esp)
uint8_t broadcast_address[] = {0x84, 0xcc, 0xa8, 0x47, 0x10, 0xfc};

// the message structure when sending data through ESP NOW
typedef struct
{
    float roll;
    float yaw;
    float pitch;
    float throttle;
} struct_message;

struct_message data;

void on_data_send(const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.print("\r\nLast Packet Send Status: Delivery fail\r\n");
    }
}

esp_now_peer_info_t peerInfo;

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_MODE_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(on_data_send);

    memcpy(peerInfo.peer_addr, broadcast_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Set the ADC attenuation to 11 dB (up to ~3.3V input)
    analogSetAttenuation(ADC_11db);

    pinMode(VRX1_PIN, INPUT);
    pinMode(VRY1_PIN, INPUT);
    pinMode(VRX2_PIN, INPUT);
    pinMode(VRY2_PIN, INPUT);

    // calibrate the joysticks
    // read the average while not moving - that is the center
    int averageX1 = readAverage(VRX1_PIN);
    int averageY1 = readAverage(VRY1_PIN);

    X1_CENTER = averageX1;
    Y1_CENTER = averageY1;

    int averageX2 = readAverage(VRX2_PIN);
    int averageY2 = readAverage(VRY2_PIN);

    X2_CENTER = averageX2;
    Y2_CENTER = averageY2;
}

void loop() {
    int averageX1 = readAverage(VRX1_PIN);
    int averageY1 = readAverage(VRY1_PIN);

    int mappedX1 = mapJoystickDelta(averageX1, X1_CENTER, X1_MIN, X1_MAX, 0, 25, 50);
    int mappedY1 = mapJoystickDelta(averageY1, Y1_CENTER, Y1_MIN, Y1_MAX, -25, 0, 25);

    int averageX2 = readAverage(VRX2_PIN);
    int averageY2 = readAverage(VRY2_PIN);

    int mappedX2 = mapJoystickDelta(averageX2, X2_CENTER, X2_MIN, X2_MAX, -25, 0, 25);
    int mappedY2 = mapJoystickDelta(averageY2, Y2_CENTER, Y2_MIN, Y2_MAX, -25, 0, 25);

    // we want the values as percentages so we convert them to floats
    data.pitch = mappedX2 / (float)100;
    data.roll = mappedY2 / (float)100;
    data.throttle = mappedX1 / (float)100;
    data.yaw = mappedY1 / (float)100;

    // Serial.printf("T: %+.2f; Y: %+.2f; P: %+.2f; R: %+.2f\n", data.throttle, data.yaw, data.pitch, data.roll);

    esp_err_t result = esp_now_send(broadcast_address, (uint8_t*)&data, sizeof(data));

    if (result != ESP_OK) {
        Serial.println("Error sending the data");
    }
    // else {
    //   Serial.println("Sent with success");
    // }

    // do nothing until it's time for the next loop (every [PERIOD] seconds)
    while (micros() - loopTimer < PERIOD * 1000000);
    loopTimer = micros();
}