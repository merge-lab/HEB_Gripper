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

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(2000);
  digitalWrite(2, LOW);
}

void loop() {
  digitalWrite(2, !digitalRead(2));
  delay(200);
}