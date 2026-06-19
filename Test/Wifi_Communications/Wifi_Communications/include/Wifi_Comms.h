#ifndef WIFI_COMMS_H
#define WIFI_COMMS_H

#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* 
Contains all data sent in telemetry
Add any new fields wanted to this
*/
struct telemetry_data
{
    // finger 1 voltage readings
    float f1s1_voltage;
    float f1s2_voltage;
    float f1s3_voltage;
    // finger 2 voltage readings
    float f2s1_voltage;
    float f2s2_voltage;
    float f2s3_voltage;
    // other data
    int time_stamp;
};

class Wifi_Communications
{
public:
    /* 
    Constructor
        ssid_name - the set name of the wifi access point
        wifi_password - the set password of the access point
    */
    Wifi_Communications(const char* ssid_name, const char* wifi_password);

    /*
    Updates the current telemetry data buffer
    */
    void update_telemetry_buffer(telemetry_data td);



private:
    const char* SSID;       // network name
    const char* PASSWORD;   // network password
    WebServer server;       // the webserver being used
    telemetry_data buffer;  // the most recently updated telemetry data

    // prevents race condition between buffer update and telemetry send
    SemaphoreHandle_t buffer_mutex;

    /*
    Initializes webserver's routes. 
    NOTE: all new functions should be added here with their triggers
    */
    void setup_routes();

    /*
    Handles simple pings for testing
    triggered when "ping" is recieved
    */
    void handle_ping();

    /*
    Sends our telemetry data
    triggered when "telemetry" is recieved
    */
    void send_telemetry();

};

#endif