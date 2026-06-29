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
  Serial.println("Commands:");
  Serial.println("  s <k>        spring base (stiffness)");
  Serial.println("  c <t>        constant-force base (torque, signed)");
  Serial.println("  f <hz>       set vibration frequency");
  Serial.println("  a <amp>      set vibration amplitude (0 = off)");
  Serial.println("  v <hz> <amp> set vibration freq + amp together");
  Serial.println("  sweep        toggle auto frequency sweep");
}

enum TestMode { MODE_SPRING, MODE_CONSTANT };

// Frequencies (Hz) stepped through in auto-sweep mode.
const int SWEEP_FREQS[] = {10, 20, 30, 50, 75, 100, 150, 200};
const int SWEEP_COUNT   = sizeof(SWEEP_FREQS) / sizeof(SWEEP_FREQS[0]);
const int SWEEP_HOLD    = 8;   // ~200 ms loop cycles to hold each step (~1.6 s)

// Send the current vibration setting and report the ACK.
void applyVibrate(int freq, float amp) {
  Response_Packet r;
  if (motor.vibrate(freq, amp, &r))
    Serial.printf("  -> vibrate %d Hz, amp %.3f\n", freq, amp);
  else
    Serial.println("  -> vibrate no ACK (rejected? freq must be 0..400)");
}

void loop() {
  static TestMode mode = MODE_SPRING;   // boot with a live spring
  static float value   = 1.0f;
  static int   vibFreq = 0;
  static float vibAmp  = 0.0f;
  static bool  sweep   = false;
  static int   sweepIdx = 0, sweepTick = 0;

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.equalsIgnoreCase("sweep")) {
      sweep = !sweep;
      sweepIdx = 0;
      sweepTick = 0;
      if (sweep && vibAmp <= 0.0f) vibAmp = 0.2f;   // need some amplitude to feel it
      if (!sweep) applyVibrate(0, 0.0f);            // stop the buzz when turning off
      Serial.printf("sweep %s (amp %.2f)\n", sweep ? "ON" : "OFF", vibAmp);
    }
    else if (line.length() > 0) {
      char ch = line.charAt(0);
      String arg = line.substring(1);
      arg.trim();

      if (ch == 's' || ch == 'S') {
        mode = MODE_SPRING; value = arg.toFloat();
        Serial.printf("base = spring %.3f\n", value);
      } else if (ch == 'c' || ch == 'C') {
        mode = MODE_CONSTANT; value = arg.toFloat();
        Serial.printf("base = constant %.3f\n", value);
      } else if (ch == 'f' || ch == 'F') {
        sweep = false; vibFreq = arg.toInt(); applyVibrate(vibFreq, vibAmp);
      } else if (ch == 'a' || ch == 'A') {
        vibAmp = arg.toFloat(); applyVibrate(vibFreq, vibAmp);
      } else if (ch == 'v' || ch == 'V') {
        sweep = false;
        int sp = arg.indexOf(' ');
        vibFreq = arg.toInt();
        vibAmp  = (sp >= 0) ? arg.substring(sp + 1).toFloat() : vibAmp;
        applyVibrate(vibFreq, vibAmp);
      } else {
        value = line.toFloat();
        Serial.printf("base value = %.3f\n", value);
      }
    }
  }

  // Auto-sweep: advance through SWEEP_FREQS, holding the current amplitude.
  if (sweep && vibAmp > 0.0f) {
    if (sweepTick == 0) {
      vibFreq = SWEEP_FREQS[sweepIdx];
      applyVibrate(vibFreq, vibAmp);
    }
    if (++sweepTick >= SWEEP_HOLD) {
      sweepTick = 0;
      sweepIdx = (sweepIdx + 1) % SWEEP_COUNT;
    }
  }

  // Re-send the active base command each cycle (keeps it set) and stream telemetry.
  Response_Packet resp;
  bool ok = (mode == MODE_SPRING) ? motor.set_spring_force(value, &resp)
                                  : motor.set_constant_force(value, &resp);
  if (ok) {
    float pos_deg, torque;
    memcpy(&pos_deg, &resp.encoder_value, sizeof(pos_deg));
    memcpy(&torque,  &resp.motor_value,  sizeof(torque));
    Serial.printf("[%s %.2f | vib %dHz %.2f%s] pos=%+.2f torque=%+.3f\n",
                  mode == MODE_SPRING ? "spr" : "con", value,
                  vibFreq, vibAmp, sweep ? " SWEEP" : "",
                  pos_deg, torque);
  } else {
    Serial.println("no ACK (base rejected or timed out)");
  }

  delay(200);
}



/*
c 0.3 — set a steady push to feel the buzz against
a 0.2 — turn vibration on at amplitude 0.2
f 20, f 50, f 100, f 150 — step the frequency; amplitude stays put so you're comparing pitch at constant strength
a 0.4 then step f again — compare strength
sweep — hands-free walk through the frequency list; sweep again to stop
f 0 or a 0 — buzz off, base remains
*/