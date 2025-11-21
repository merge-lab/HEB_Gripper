#include "Arduino.h"
#include <HardwareSerial.h>
#include "communication.h"

void process_data(char* buffer, size_t* len);

void setup() {
    Serial.begin(1843200);
    Serial2.begin(1843200, SERIAL_8N1, 16, 17); // RX = 16, TX = 17
    delay(500);
    while (Serial2.available()) {
        Serial2.read();  // Discard startup garbage
    }
    init_communication(&Serial2);
    delay(500);
    Serial2.write("f\n"); // send first command to connect (for some reason the first command does not get a response)
}

void loop() {
    char rx_buffer[RX_BUFFER_SIZE];
    size_t len;
    if (get_received_line(rx_buffer, &len)) {
        Serial.print("RX: ");
        Serial.write((uint8_t*)rx_buffer, len);
        process_data(rx_buffer, &len);
    }
}

void process_data(char* buffer, size_t* len) {
    switch(buffer[0]) {
        case 'f': { // read faults
            break;
        }
        case 'v': { // recieve variables
            break;
        }
    };
}