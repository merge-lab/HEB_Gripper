#include <Arduino.h>
#include "haptic_motor_control.h"
#include "dynamixel_control.h"
#include "CLI_Control.h"
#include "tactile_sensors.h"

// I2C temp sensor (CJMCU-9808) monitoring the haptic motor - disabled for now.
// Wiring when re-enabled: ESP32 SCL=14 -> sensor SCL, ESP32 SDA=13 -> sensor SDA,
// common GND. Sensor ALERT pin wired to GPIO 7 but not read by firmware yet.

// UART2 link to the STM32 haptic motor controller.
// Wiring: ESP32 TX=17 -> STM RX, ESP32 RX=16 <- STM TX, common GND. 115200 8N1.
HapticMotorController motor(&Serial2, nullptr);

// UART1 link to the Arduino Mega / DynamixelShield gripper board.
// Wiring: ESP32 TX=21 -> Mega RX2 (pin 17), ESP32 RX=35 <- [level shifter] <- Mega TX2 (pin 16),
// common GND. 1,000,000 8N1.
DynamixelController gripper(&Serial1);

void setup() {
  pinMode(2, OUTPUT);                          // onboard LED used as a link indicator
  // Native USB (USB Serial/JTAG): the baud rate is ignored, data moves at USB
  // full speed. A larger TX buffer lets 'tac 2000' stream (~120 KB/s)
  // without stalling the loop.
  Serial.begin(115200);
  Serial.setTxBufferSize(16384);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // link to the STM32
  tactile_begin();                             // ADC pins 4,5,6,8,9,10 for T1..T6
  gripper.begin();                             // link to the Arduino gripper board (GPIO 35/21)
  delay(500);

  CLI_Setup(&motor, &gripper);
}

void loop()
{
  CLI(&motor, &gripper);
  // slip_mapping_test();
}
