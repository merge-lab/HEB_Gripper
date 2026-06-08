#include "Arduino.h"
#include "forte.h"
#include "Wifi_Comms.h"
#include <FT_Device.h>

#define RXD2 16
#define TXD2 17

void setup() {
    // Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    delay(1500);

    // Serial.println("Setting SPRING mode...");

    // Set spring mode ONCE
    Serial2.print("m 1\n");
    delay(200);

    // Serial.println("Sending spring torque loop...");
}

void loop() {
    // small constant "spring stiffness"
    // increase/decrease this to feel stiffness
    float k = 1.5;

    Serial2.printf("t %f\n", k);

    // print any STM32 responses
    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    delay(20);  // 50 Hz update rate
}