// // ESP32 Arduino code
// // 6 tactile channels @ 1000 Hz
// // Regressor updates @ 100 Hz
// // Fake Ridge / fake MLP weights for hookup testing
// // No extra library needed
// #include <Arduino.h>

// const int numSensors = 6;
// const int sensorPins[numSensors] = {4, 5, 6, 8, 9, 10};

// // =========================
// // Choose regressor here
// // =========================
// // Ridge:
// #define USE_RIDGE 1
// #define USE_MLP   0

// // MLP:
// // #define USE_RIDGE 0
// // #define USE_MLP   1

// // =========================
// // ADC settings
// // =========================
// const float refVoltage    = 3.3;
// const int   adcResolution = 4095;
// const float adcFactor = refVoltage / (float)adcResolution;

// const float Vs = 3.0;
// const float minVoltage = 0.1 * Vs;
// const float maxVoltage = 0.9 * Vs;

// // =========================
// // Timing
// // =========================
// const unsigned long sensorIntervalMicros = 1000;  // 1000 Hz sensor read
// const int regressorEveryN = 10;                   // 100 Hz regressor update

// int sampleCounter = 0;

// // =========================
// // Sensor data
// // =========================
// int raw[numSensors];
// float mapped[numSensors];

// float forceOutput = 0.0;

// // ======================================================
// // Fake Ridge regressor
// // force = bias + w1*T1 + ... + w6*T6
// // ======================================================

// float ridgeBias = 0.5;

// float ridgeW[numSensors] = {
//   1.0,
//   2.0,
//   3.0,
//   4.0,
//   5.0,
//   6.0
// };

// float ridgePredict(const float x[numSensors]) {
//   float y = ridgeBias;

//   for (int i = 0; i < numSensors; i++) {
//     y += ridgeW[i] * x[i];
//   }

//   return y;
// }

// // ======================================================
// // Fake TinyML MLP regressor
// // Architecture: 6 -> 8 -> 1
// // ======================================================

// const int hiddenSize = 8;

// float W1[hiddenSize][numSensors] = {
//   { 1.0,  0.0,  0.0,  0.0,  0.0,  0.0},
//   { 0.0,  1.0,  0.0,  0.0,  0.0,  0.0},
//   { 0.0,  0.0,  1.0,  0.0,  0.0,  0.0},
//   { 0.0,  0.0,  0.0,  1.0,  0.0,  0.0},
//   { 0.0,  0.0,  0.0,  0.0,  1.0,  0.0},
//   { 0.0,  0.0,  0.0,  0.0,  0.0,  1.0},
//   { 0.5,  0.5,  0.5,  0.5,  0.5,  0.5},
//   {-0.5, -0.5, -0.5, -0.5, -0.5, -0.5}
// };

// float b1[hiddenSize] = {
//   0.1,
//   0.2,
//   0.3,
//   0.4,
//   0.5,
//   0.6,
//   0.0,
//   0.0
// };

// float W2[hiddenSize] = {
//   0.4,
//   0.6,
//   0.8,
//   1.0,
//   1.2,
//   1.4,
//   0.5,
//  -0.3
// };

// float b2 = 0.25;

// float relu(float v) {
//   return v > 0.0 ? v : 0.0;
// }

// float mlpPredict(const float x[numSensors]) {
//   float h[hiddenSize];

//   for (int i = 0; i < hiddenSize; i++) {
//     float sum = b1[i];

//     for (int j = 0; j < numSensors; j++) {
//       sum += W1[i][j] * x[j];
//     }

//     h[i] = relu(sum);
//   }

//   float y = b2;

//   for (int i = 0; i < hiddenSize; i++) {
//     y += W2[i] * h[i];
//   }

//   return y;
// }

// // ======================================================
// // Unified prediction function
// // ======================================================

// float predictForce(const float x[numSensors]) {
// #if USE_RIDGE
//   return ridgePredict(x);
// #elif USE_MLP
//   return mlpPredict(x);
// #else
//   return 0.0;
// #endif
// }

// void setup() {
//   Serial.begin(2000000);
//   delay(500);
// }

// void loop() {
//   unsigned long loopStart = micros();

//   // =========================
//   // Read sensors @ 1000 Hz
//   // =========================
//   for (int i = 0; i < numSensors; i++) {
//     raw[i] = analogRead(sensorPins[i]);
//   }

//   // =========================
//   // Convert ADC to mapped values
//   // =========================
//   for (int i = 0; i < numSensors; i++) {
//     mapped[i] = (((raw[i] * adcFactor) - minVoltage) /
//                  (maxVoltage - minVoltage)) * 4.0 - 2.0;
//   }

//   // =========================
//   // Update force @ 100 Hz
//   // =========================
//   if (sampleCounter % regressorEveryN == 0) {
//     forceOutput = predictForce(mapped);
//   }

//   sampleCounter++;

//   // =========================
//   // Print every loop
//   // Force repeats between 100 Hz updates
//   // =========================
//   Serial.printf(
//     "T1:%.3f,T2:%.3f,T3:%.3f,T4:%.3f,T5:%.3f,T6:%.3f,Force:%.3f\n",
//     mapped[0], mapped[1], mapped[2],
//     mapped[3], mapped[4], mapped[5],
//     forceOutput);

//   // =========================
//   // Maintain 1000 Hz loop
//   // =========================
//   unsigned long elapsed = micros() - loopStart;

//   if (elapsed < sensorIntervalMicros) {
//     delayMicroseconds(sensorIntervalMicros - elapsed);
//   }
// }

// #include <Arduino.h>
// #include <Dynamixel2Arduino.h>

// #define DXL_SERIAL Serial2
// #define DXL_DIR_PIN 18

// Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   Serial.println("A");

//   DXL_SERIAL.begin(57600, SERIAL_8N1, 16, 17);
//   Serial.println("B");

//   dxl.begin(57600);
//   Serial.println("C");

//   dxl.setPortProtocolVersion(2.0);

//   Serial.println("D");

//   if (dxl.ping(1)) {
//     Serial.println("FOUND");
//   } else {
//     Serial.println("NOT FOUND");
//   }
// }

// void loop() {
//   Serial.println("loop");
//   delay(1000);
// }

#include <Arduino.h>
#include "haptic_motor_control.h"

// UART2 link to the STM32 motor controller.
// Wiring: ESP32 TX=17 -> STM RX, ESP32 RX=16 <- STM TX, common GND. 115200 8N1.
HapticMotorController motor(&Serial2);

void setup() {
  pinMode(2, OUTPUT);                          // onboard LED used as a link indicator
  Serial.begin(115200);                        // USB serial for debug output
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // link to the STM32
  delay(500);

  bool pong = motor.ping_controller(500);
  digitalWrite(2, pong ? HIGH : LOW);
  Serial.println(pong ? "link up (PONG)" : "no PONG - check link");
  Serial.println("Type a spring constant (e.g. 1.0) and press enter:");
}

enum TestMode { MODE_SPRING, MODE_CONSTANT };

void loop() {
  static TestMode mode = MODE_SPRING;   // boot with a live spring
  static float value = 1.0f;

  // Serial input: "s <val>" = spring stiffness, "c <val>" = constant torque,
  // a bare number keeps the current mode and just changes the value.
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      char c = line.charAt(0);
      if (c == 's' || c == 'S')      { mode = MODE_SPRING;   value = line.substring(1).toFloat(); }
      else if (c == 'c' || c == 'C') { mode = MODE_CONSTANT; value = line.substring(1).toFloat(); }
      else                           { value = line.toFloat(); }
      Serial.printf("mode=%s value=%.3f\n",
                    mode == MODE_SPRING ? "spring" : "constant", value);
    }
  }

  // Re-send the active command each cycle (keeps it set) and stream telemetry.
  Response_Packet resp;
  bool ok = (mode == MODE_SPRING) ? motor.set_spring_force(value, &resp)
                                  : motor.set_constant_force(value, &resp);
  if (ok) {
    float pos_deg, torque;
    memcpy(&pos_deg, &resp.encoder_value, sizeof(pos_deg));
    memcpy(&torque,  &resp.motor_value,  sizeof(torque));
    Serial.printf("[%s %.3f] pos=%+.2f deg  torque=%+.3f\n",
                  mode == MODE_SPRING ? "spr" : "con", value, pos_deg, torque);
  } else {
    Serial.println("no ACK (rejected or timed out)");
  }

  delay(200);
}