// Siqi's old logging software, stored for reference
// const int numSensors = 6;
// const int sensorPins[numSensors] = {4, 5, 6, 8, 9, 10};  // ADC-capable pins (avoid GPIO0)

// const float refVoltage    = 3.3;   // ADC reference voltage
// const int   adcResolution = 4095;  // 12-bit ADC
// const float adcFactor = refVoltage / (float)adcResolution; 

// // Sensor calibration: original voltage range (scaled from 5V to 3.3V)
// const float minVoltage = 0.25 * (3.3 / 5.0);   // ~0.165 V
// const float maxVoltage = 4.25 * (3.3 / 5.0);     // ~2.805 V

// // Desired loop period (500 µs = 0.5 ms)
// // Test with 1000 hz
// const unsigned long loopIntervalMicros = 1000;
// unsigned long readingStart;

// int  raw[numSensors];
// // float volt[numSensors];
// float mapped[numSensors];

// void setup() {
//   Serial.begin(2000000); // High baud rate for faster printing
//   readingStart = micros();
//   delay(500);
//   delayMicroseconds(loopIntervalMicros - ((micros() - readingStart) % loopIntervalMicros));
// }

// void loop() {
//   // unsigned long loopStart = micros();
//   unsigned long loopStart = micros();

//   // ----- Sensor Reading Step -----
//   for (int i = 0; i < numSensors; i++) {
//     raw[i] = analogRead(sensorPins[i]);
//   }
//   // unsigned long tRead = micros();

//   // ----- Conversion Step -----
//   for (int i = 0; i < numSensors; i++) {
//     // Convert raw ADC value to voltage (0 to refVoltage)
//     // volt[i] = (raw[i] / (float)adcResolution) * refVoltage;
//     // Clamp the voltage to the sensor's calibrated range
//     // if (volt[i] < minVoltage) volt[i] = minVoltage;
//     // if (volt[i] > maxVoltage) volt[i] = maxVoltage;
//     // Map the voltage [minVoltage, maxVoltage] to [-1.0, 1.0]
//     // mapped[i] = ((((raw[i] / (float)adcResolution) * refVoltage) - minVoltage) / (maxVoltage - minVoltage)) * 2.0 - 1.0;
//     mapped[i] = (((raw[i] * adcFactor) - minVoltage) / (maxVoltage - minVoltage)) * 2.0 - 1.0;
//   }
//   // unsigned long tConv = micros();

//   // delayMicroseconds(100);
//   // ----- Printing Step -----
//   // Print only the mapped values in a compact line
//   // Serial.printf("T1:%.3f, T2:%.3f, T3:%.3f\n", mapped[0], mapped[1], mapped[2]);
//   // Serial.printf("T1:%.3f, T2:%.3f, T3:%.3f, T4:%.3f\n", mapped[0], mapped[1], mapped[2], mapped[3]);
//   // Serial.printf(F("T1:%.3f, T2:%.3f, T3:%.3f, T4:%.3f\n"),
//   //             mapped[0], mapped[1], mapped[2], mapped[3]);
//   Serial.printf("T1:%.3f,T2:%.3f,T3:%.3f,T4:%.3f,T5:%.3f,T6:%.3f\n", 
//     mapped[0], mapped[1], mapped[2], mapped[3], mapped[4], mapped[5]);
//   // unsigned long tPrint = micros();

//   // Print latency information for each step (in µs)
//   // Serial.printf("Latencies (µs): R=%lu, C=%lu, P=%lu, Total=%lu\n",
//   // Serial.printf("P=%lu\n",
//   // Serial.printf("R=%lu, C=%lu, P=%lu\n",
//   //               tRead - loopStart,
//   //               tConv - tRead,
//   //               tPrint - tConv);

//   // ----- Maintain Loop Timing -----
//   unsigned long elapsed = micros() - loopStart;
//   if (elapsed < loopIntervalMicros) {
//     delayMicroseconds(loopIntervalMicros - ((micros() - readingStart) % loopIntervalMicros));
//   }
//   // else{
//   //     Serial.println("elapsed");
//   // }
// }
