#include <Arduino.h>
#include "haptic_motor_control.h"
#include "dynamixel_control.h"
#include "CLI_Control.h"

// UART2 link to the STM32 haptic motor controller.
// Wiring: ESP32 TX=17 -> STM RX, ESP32 RX=16 <- STM TX, common GND. 115200 8N1.
HapticMotorController motor(&Serial2);

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

/*
Try various rates of vibration, see what people think sliprate is.
Hypothesis: higher slip rate = lower frequency. 
  Therefore low sliprate = high frequency and eventually blends into 0 freq 
Experiment: will do 3 frequencies + no frequency. Users can rate very fast slip
  , low slip, very low slip, no slip. Will have no force in between readings.
  will run each for 3 seconds with a 10 second break
*/
void slip_mapping_test()
{
  unsigned long end;

  // Segment 1: constant force only, 3 s
  motor.vibrate(0, 0.0f, NULL);
  end = millis() + 3000;
  while (millis() < end) { motor.set_constant_force(-1.5f, NULL); delay(200); }

  // Break: relax 10 s
  end = millis() + 10000;
  while (millis() < end) { motor.relax(NULL); delay(200); }

  // Segment 2: high frequency (80 Hz) + constant force, 3 s
  motor.vibrate(80, 0.5f, NULL);
  end = millis() + 5000;
  while (millis() < end) { motor.set_constant_force(-1.5f, NULL); delay(200); }

  // Break
  end = millis() + 2000;
  while (millis() < end) { motor.relax(NULL); delay(200); }

  // Segment 3: mid frequency (50 Hz) + constant force, 3 s
  motor.vibrate(50, 0.5f, NULL);
  end = millis() + 5000;
  while (millis() < end) { motor.set_constant_force(-1.5f, NULL); delay(200); }

  // Break
  end = millis() + 2000;
  while (millis() < end) { motor.relax(NULL); delay(200); }

  // Segment 4: low frequency (25 Hz) + constant force, 3 s
  motor.vibrate(25, 0.5f, NULL);
  end = millis() + 5000;
  while (millis() < end) { motor.set_constant_force(-1.5f, NULL); delay(200); }

  motor.relax(NULL);

  delay(5000);
}

void loop()
{
  CLI(&motor, &gripper);
  // slip_mapping_test();
}
