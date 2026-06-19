#include <Arduino.h>
#include <HX711.h>

HX711 loadcell;

// 1. HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

const long LOADCELL_DIVIDER = 43733;

void setup() {
  Serial.begin(115200);
  // 3. Initialize library
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  loadcell.set_scale(LOADCELL_DIVIDER);
  loadcell.tare();
}

void loop() {
  Serial.print("Weight: ");
  Serial.println(loadcell.get_units(10));
  delay(200);
}