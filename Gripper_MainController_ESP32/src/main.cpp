// #include "Arduino.h"
// #include <HardwareSerial.h>
// #include "communication.h"

// void process_data(char* buffer, size_t* len);

// void setup() {
//     Serial.begin(115200);
//     Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX = 16, TX = 17
//     delay(500);
//     while (Serial2.available()) {
//         Serial2.read();  // Discard startup garbage
//     }
//     init_communication(&Serial2);
//     delay(500);
//     Serial2.write("f\n"); // send first command to connect (for some reason the first command does not get a response)
// }

// String line = "";

// void loop() {
//     while (Serial2.available()) {
//         char c = Serial2.read();

//         if (c == '\n') {
//             Serial.println("STM32: " + line);
//             line = "";
//         } else {
//             line += c;
//         }
//     }
// }


// void process_data(char* buffer, size_t* len) {
//     switch(buffer[0]) {
//         case 'f': { // read faults
//             break;
//         }
//         case 'v': { // recieve variables
//             break;
//         }
//     };
// }


#include "Arduino.h"
#include "forte.h"

#define RXD2 16
#define TXD2 17

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    delay(1500);

    Serial.println("Setting SPRING mode...");

    // Set spring mode ONCE
    Serial2.print("m 1\n");
    delay(200);

    Serial.println("Sending spring torque loop...");

    ForteFinger x (1, 2, 3, "hi");
}

void loop() {
    // small constant "spring stiffness"
    // increase/decrease this to feel stiffness
    float k = 0.3;

    Serial2.printf("t %f\n", k);

    // print any STM32 responses
    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    delay(20);  // 50 Hz update rate
}