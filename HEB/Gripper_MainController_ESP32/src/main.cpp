#include <Arduino.h>
#include "haptic_motor_control.h"
#include "CLI_Control.h"
#include "FORTE_Fingers.h"

// UART2 link to the STM32 motor controller.
// Wiring: ESP32 TX=17 -> STM RX, ESP32 RX=16 <- STM TX, common GND. 115200 8N1.
HapticMotorController motor(&Serial2);

// Tactile sensor pins — edit here if your wiring changes.
static const int SENSOR_PINS[6] = {4, 5, 6, 8, 9, 10};
FORTE_Fingers forte_fingers(SENSOR_PINS);

void setup() {
  pinMode(2, OUTPUT);                          // onboard LED used as a link indicator
  Serial.begin(115200);                        // USB serial for debug output
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // link to the STM32
  delay(500);

  // Slip detection + force estimation runs on core 0 at 1 kHz.
  // Core 1 (this loop) reads results every 200 ms via get_latest().
  xTaskCreatePinnedToCore(forte_task, "forte", 4096, &forte_fingers, 2, nullptr, 0);

  CLI_Setup(&motor, &forte_fingers);
}

void loop()
{
  CLI(&motor, &forte_fingers);
}



