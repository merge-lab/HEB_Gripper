#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <HardwareSerial.h>

#define RX_BUFFER_SIZE 128

typedef enum {
    CONSTANT,
    SPRING
} ControlType;

void init_communication(HardwareSerial *serial);
bool get_received_line(char *buffer, size_t *len);
void send_torque(float torque);
void change_mode(ControlType control_type);



#endif