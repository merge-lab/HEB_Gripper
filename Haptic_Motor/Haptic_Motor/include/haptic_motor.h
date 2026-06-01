#ifndef HAPTIC_MOTOR_H
#define HAPTIC_MOTOR_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <HardwareSerial.h>

/*
Type of control the haptics run under
*/
typedef enum {
    CONSTANT,
    SPRING
} ControlType;

/*
Class to control communication to the haptic motor
*/
class HapticMotor
{
public:
private:
    SemaphoreHandle_t data_ready = NULL;

    // helper methods from the original version
    void init_communication(HardwareSerial *serial);
    bool get_recieved_line(char* buffer, size_t *len);
    void send_torque(float torque);
    void change_mode(ControlType control_type);
};


#endif