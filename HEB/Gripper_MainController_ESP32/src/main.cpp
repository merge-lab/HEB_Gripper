#include <Arduino.h>
#include "haptic_motor_control.h"
#include "dynamixel_control.h"
#include "CLI_Control.h"
#include "Master_Side.h"
#include "temp_senor.h"

// I2C temp sensor (CJMCU-9808) monitoring the haptic motor; pauses force output
// via HapticMotorController::send_command() above 80C until it cools back down.
// TODO: set real SCL/SDA pins once the sensor is wired up.
CJMCU9908_Temp_Sensor haptic_temp_sensor(/*scl*/ 9, /*sda*/ 8, /*alert_temp*/ 80);

// UART2 link to the STM32 haptic motor controller.
// Wiring: ESP32 TX=17 -> STM RX, ESP32 RX=16 <- STM TX, common GND. 115200 8N1.
HapticMotorController motor(&Serial2, &haptic_temp_sensor);

// UART1 link to the Arduino Uno / DynamixelShield gripper board.
// Wiring: ESP32 TX=21 -> Uno pin 10, ESP32 RX=35 <- Uno pin 12, common GND. 115200 8N1.
DynamixelController gripper(&Serial1);

void setup() {
  pinMode(2, OUTPUT);                          // onboard LED used as a link indicator
  Serial.begin(115200);                        // USB CDC serial for debug output
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // link to the STM32
  gripper.begin();                             // link to the Arduino gripper board (GPIO 35/21)
  delay(500);

  CLI_Setup(&motor, &gripper);
}

void loop()
{
  CLI(&motor, &gripper);
  // slip_mapping_test();
}
