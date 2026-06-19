#include "Wifi_Comms.h"

Wifi_Communications::Wifi_Communications(const char* ssid_name, const char* 
    wifi_password) : SSID(ssid_name), PASSWORD(wifi_password), server(80)
{
    buffer_mutex = xSemaphoreCreateMutex();
    setup_routes();
    server.begin();
    telemetry_data buffer = {};
}

void Wifi_Communications::setup_routes()
{
    server.on("/ping", [this]() {
        handle_ping();
    });

    server.on("/telemetry", [this]() {
        send_telemetry();
    });
}

void Wifi_Communications::update_telemetry_buffer(telemetry_data td)
{
    // make sure send is not in progress to write
    if (xSemaphoreTake(buffer_mutex, portMAX_DELAY))
    {
        buffer = td;
        xSemaphoreGive(buffer_mutex);
    }
}

void Wifi_Communications::handle_ping()
{
    server.send(200, "text/plain", "pong");
}

void Wifi_Communications::send_telemetry()
{
    telemetry_data td;

    // copy with gaurantee of write not in progress
    if (xSemaphoreTake(buffer_mutex, portMAX_DELAY))
    {
        td = buffer;
        xSemaphoreGive(buffer_mutex);
    }

    // make it a json to send
    String json = "{";
    json += "\"f1s1_voltage\":" + String(td.f1s1_voltage, 3) + ",";
    json += "\"f1s2_voltage\":" + String(td.f1s2_voltage, 3) + ",";
    json += "\"f1s3_voltage\":" + String(td.f1s3_voltage, 3) + ",";

    json += "\"f2s1_voltage\":" + String(td.f2s1_voltage, 3) + ",";
    json += "\"f2s2_voltage\":" + String(td.f2s2_voltage, 3) + ",";
    json += "\"f2s3_voltage\":" + String(td.f2s3_voltage, 3) + ",";

    json += "\"time_stamp\":" + String(td.time_stamp);
    json += "}";

    server.send(200, "application/json", json);
}

