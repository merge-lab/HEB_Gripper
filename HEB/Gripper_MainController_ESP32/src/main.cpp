#include <Arduino.h>
#include "haptic_motor_control.h"
#include "dynamixel_control.h"
#include "CLI_Control.h"

// UART2 link to the STM32 haptic motor controller.
// Wiring: ESP32 TX=17 -> STM RX, ESP32 RX=16 <- STM TX, common GND. 115200 8N1.
HapticMotorController motor(&Serial2);

// SPI + interrupt link to the Arduino Uno / DynamixelShield gripper board.
// Wiring: ESP32 GPIO11 -> Uno D11 (MOSI), ESP32 GPIO13 <- Uno D12 (MISO, via
// resistor divider - Uno is 5V, ESP32 is not 5V-tolerant), ESP32 GPIO12 ->
// Uno D13 (SCK), ESP32 GPIO10 -> Uno D10 (SS), common GND.
// GRIPPER_DATA_READY_PIN: ESP32 GPIO40 <- Uno D9 (data-ready), via the same
// resistor-divider level shift as MISO (Uno is 5V, ESP32 is not 5V-tolerant).
// GPIO40 has no strapping/flash/USB/PSRAM conflicts on this board variant
// (ESP32-S3-DevKitC-1-N8, no PSRAM) and isn't used elsewhere in this project.
DynamixelController gripper(/*mosi*/35, /*miso*/36, /*sck*/37, /*ss*/38, /*Interrupt Pin*/40);

void setup() {
  pinMode(2, OUTPUT);                          // onboard LED used as a link indicator
  Serial.begin(115200);                        // USB CDC serial for debug output
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // link to the STM32
  gripper.begin();                             // SPI + interrupt link to the Arduino gripper board
  delay(500);

  CLI_Setup(&motor, &gripper);
}

void loop()
{
  gripper.update();
  CLI(&motor, &gripper);
  // slip_mapping_test();
}
