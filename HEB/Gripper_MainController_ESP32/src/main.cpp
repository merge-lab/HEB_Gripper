#include <Arduino.h>
#include "haptic_motor_control.h"
#include "CLI_Control.h"

// UART2 link to the STM32 motor controller.
// Wiring: ESP32 TX=17 -> STM RX, ESP32 RX=16 <- STM TX, common GND. 115200 8N1.
HapticMotorController motor(&Serial2);

void setup() {
  pinMode(2, OUTPUT);                          // onboard LED used as a link indicator
  Serial.begin(115200);                        // USB serial for debug output
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // link to the STM32
  delay(500);

  CLI_Setup(&motor);
}

void loop()
{
  CLI(&motor);

}



