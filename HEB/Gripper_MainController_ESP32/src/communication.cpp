#include "communication.h"

SemaphoreHandle_t data_ready = NULL;
char rx_line_buffer[RX_BUFFER_SIZE];
volatile size_t rx_line_length = 0;

static HardwareSerial *uart_serial = NULL;
static char rx_temp_buffer[RX_BUFFER_SIZE];
static size_t rx_temp_index = 0;

void uart_rx_task(void *parameter) {
    while (1) {
        while (uart_serial->available()) {
            char c = uart_serial->read();
            if (rx_temp_index < RX_BUFFER_SIZE - 1) {
                rx_temp_buffer[rx_temp_index++] = c;                
                if (c == '\n' || c == '\r') {
                    rx_temp_buffer[rx_temp_index] = '\0';
                    memcpy(rx_line_buffer, rx_temp_buffer, rx_temp_index);
                    rx_line_length = rx_temp_index;
                    rx_temp_index = 0;
                    xSemaphoreGive(data_ready);
                }
            } else { // somehow it overflowed
                rx_temp_index = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init_communication(HardwareSerial *serial) {
    uart_serial = serial;
    data_ready = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(uart_rx_task, "UART_RX", 4096, NULL, 2, NULL, 1);
}

bool get_received_line(char *buffer, size_t *len) {
    if (xSemaphoreTake(data_ready, 0) == pdTRUE) {
        memcpy(buffer, rx_line_buffer, rx_line_length);
        *len = rx_line_length;
        return true;
    }
    return false;
}

// Note when in linear torque mode, the value of the torque provided should be in units of torque per degree of rotation

void send_torque(float torque) {
    Serial2.printf("t %f\n", torque);
}

void change_mode(ControlType control_type) {
    Serial2.printf("m %d\n", control_type);
}