#include <WiFi.h>
#include <esp_now.h>

typedef struct
{
    float roll;
    float yaw;
    float pitch;
    float throttle;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
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

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_MODE_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {}
