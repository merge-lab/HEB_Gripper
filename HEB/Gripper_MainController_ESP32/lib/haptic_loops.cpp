// Rishit's old haptic loop code, for reference

// #include "Arduino.h"
// #include "forte.h"
// #include "Wifi_Comms.h"
// // #include "FT_Device.h"

// #define RXD2 16
// #define TXD2 17

// // FT_Device left("left");
// // FT_Device right("right");
// // const float refVoltage    = 3.3;   // ADC reference voltage
// // const int   adcResolution = 4095;  // 12-bit ADC
// // const float adcFactor = refVoltage / (float)adcResolution; 

// // // Sensor calibration: original voltage range (scaled from 5V to 3.3V)
// // const float minVoltage = 0.25 * (3.3 / 5.0);   // ~0.165 V
// // const float maxVoltage = 4.25 * (3.3 / 5.0);     // ~2.805 V


// void setup() {

//     // Serial.begin(115200);
//     Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

//     delay(1500);

//     // Serial.println("Setting SPRING mode...");

//     // Set spring mode ONCE
//     Serial2.print("m 1\n");
//     delay(200);

//     // Serial.println("Sending spring torque loop...")
//     // Serial.begin(9600);
// }

// void loop() {

//     // small constant "spring stiffness"
//     // increase/decrease this to feel stiffness
//     float k = 1.5;

//     Serial2.printf("t %f\n", k);

//     // print any STM32 responses
//     while (Serial2.available()) {
//         Serial.write(Serial2.read());
//     }

//     // delay(20);  // 50 Hz update rate
//     // Serial.println("Hello");

// }

