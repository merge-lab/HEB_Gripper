#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DynamixelShield.h>
#include "dynamixel_protocol.h"

// ---- Dynamixel config -------------------------------------------------------
DynamixelShield dxl;
const float     DXL_PROTOCOL  = 2.0;
const long      DXL_BAUD      = 1000000;
const uint8_t   ID_LEFT       = 0;
const uint8_t   ID_RIGHT      = 1;

// Default profile velocity for smooth tracking. Explicit speed in a move
// command overrides this. 0 = max speed (no limit).
#define DEFAULT_PROFILE_VEL_RPM  0.0f

// Goal Position register for Protocol 2.0 servos (XM, XH, XL430, MX 2.0).
// 4096 counts per revolution; raw = degrees * 4096 / 360.
#define GOAL_POS_ADDR  116
#define GOAL_POS_LEN   4

// ---- ESP32 serial link ------------------------------------------------------
// Hardware Serial (pins 0/1) is owned by DynamixelShield.
// Pin 2 is DXL_DIR_PIN — do not use for SoftwareSerial.
// Wire: Arduino pin 10 <- ESP32 TX (GPIO21),  Arduino pin 12 -> ESP32 RX (GPIO35).
SoftwareSerial espSerial(10, 12);
const long     ESP_BAUD = 57600;

// ---- Heartbeat safety -------------------------------------------------------
#define HEARTBEAT_TIMEOUT_MS 500UL

static unsigned long last_cmd_ms   = 0;
static bool          servos_active = false;

// ---- Packet receiver state machine ------------------------------------------
static uint8_t rx_buf[sizeof(Dyn_Command_Packet)];
static uint8_t rx_idx    = 0;
static bool    rx_active = false;

// ---- LED --------------------------------------------------------------------
const uint8_t LED_PIN = 13;

static void blink_led(int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH); delay(on_ms);
        digitalWrite(LED_PIN, LOW);  delay(off_ms);
    }
}

// ---- Sync Write setup -------------------------------------------------------
// Sends goal positions to both servos in a single packet so they start
// moving at exactly the same time.

typedef struct { int32_t position; } __attribute__((packed)) sync_pos_t;

static sync_pos_t                      sync_pos[2];
static DYNAMIXEL::InfoSyncWriteInst_t  sw_info;
static DYNAMIXEL::XELInfoSyncWrite_t   sw_xels[2];

static void init_sync_write()
{
    sw_info.packet.p_buf        = nullptr;
    sw_info.packet.is_completed = false;
    sw_info.p_xels              = sw_xels;
    sw_info.addr                = GOAL_POS_ADDR;
    sw_info.addr_length         = GOAL_POS_LEN;
    sw_info.xel_count           = 2;

    sw_xels[0].id     = ID_LEFT;
    sw_xels[0].p_data = (uint8_t*)&sync_pos[0].position;
    sw_xels[1].id     = ID_RIGHT;
    sw_xels[1].p_data = (uint8_t*)&sync_pos[1].position;
}

static inline int32_t deg_to_raw(float deg)
{
    return (int32_t)(deg * (4096.0f / 360.0f));
}

// Move both servos simultaneously via a single Sync Write packet.
static bool sync_move(float left_deg, float right_deg)
{
    sync_pos[0].position        = deg_to_raw(left_deg);
    sync_pos[1].position        = deg_to_raw(right_deg);
    sw_info.packet.is_completed = false;
    return dxl.syncWrite(&sw_info);
}

// ---- Profile-velocity helper ------------------------------------------------
static void set_profile_velocity(uint8_t id, float speed_rpm)
{
    if (speed_rpm <= 0.0f) return;
    uint32_t raw = (uint32_t)(speed_rpm / 0.229f);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, raw);
}

// ---- Response sender --------------------------------------------------------
static void send_response(Dyn_Response resp_code)
{
    Dyn_Response_Packet pkt;
    pkt.startByte = DYN_START_BYTE;
    pkt.response  = (uint8_t)resp_code;

    float left_mA  = dxl.getPresentCurrent(ID_LEFT,  UNIT_MILLI_AMPERE);
    float right_mA = dxl.getPresentCurrent(ID_RIGHT, UNIT_MILLI_AMPERE);

    memcpy(&pkt.left_motor_current,  &left_mA,  sizeof(pkt.left_motor_current));
    memcpy(&pkt.right_motor_current, &right_mA, sizeof(pkt.right_motor_current));

    pkt.endByte = DYN_END_BYTE;
    espSerial.write((const uint8_t*)&pkt, sizeof(pkt));
}

// ---- Command handler --------------------------------------------------------
static void handle_command(const Dyn_Command_Packet* cmd)
{
    last_cmd_ms = millis();

    switch (cmd->command)
    {
        case DYN_PING:
            send_response(DYN_PONG);
            break;

        case DYN_MOVE_SERVO:
        {
            float left_deg, right_deg, left_spd, right_spd;
            memcpy(&left_deg,  &cmd->left_degrees,  sizeof(left_deg));
            memcpy(&right_deg, &cmd->right_degrees, sizeof(right_deg));
            memcpy(&left_spd,  &cmd->left_speed,    sizeof(left_spd));
            memcpy(&right_spd, &cmd->right_speed,   sizeof(right_spd));

            if (!servos_active) {
                dxl.torqueOn(ID_LEFT);
                dxl.torqueOn(ID_RIGHT);
                servos_active = true;
            }

            // Override profile velocity only when explicitly requested.
            set_profile_velocity(ID_LEFT,  left_spd);
            set_profile_velocity(ID_RIGHT, right_spd);

            bool ok = sync_move(left_deg, right_deg);
            send_response(ok ? DYN_ACK : DYN_FAILURE);
            break;
        }

        case DYN_MOVE_SERVO_FAST:
        {
            // High-rate control path: move servos, send no response.
            // Skipping send_response avoids SoftwareSerial TX disabling
            // interrupts, which would corrupt the incoming RX stream.
            float left_deg, right_deg;
            memcpy(&left_deg,  &cmd->left_degrees,  sizeof(left_deg));
            memcpy(&right_deg, &cmd->right_degrees, sizeof(right_deg));

            if (!servos_active) {
                dxl.torqueOn(ID_LEFT);
                dxl.torqueOn(ID_RIGHT);
                servos_active = true;
            }

            sync_move(left_deg, right_deg);
            break;
        }

        case DYN_STOP:
            dxl.torqueOff(ID_LEFT);
            dxl.torqueOff(ID_RIGHT);
            servos_active = false;
            send_response(DYN_ACK);
            break;

        default:
            send_response(DYN_MALFORMED);
            break;
    }
}

// ---- Packet accumulator -----------------------------------------------------
static void poll_esp_serial()
{
    while (espSerial.available())
    {
        uint8_t b = espSerial.read();

        if (!rx_active) {
            if (b == (uint8_t)DYN_START_BYTE) {
                rx_buf[0] = b;
                rx_idx    = 1;
                rx_active = true;
            }
        } else {
            rx_buf[rx_idx++] = b;

            if (rx_idx == sizeof(Dyn_Command_Packet)) {
                rx_active = false;
                rx_idx    = 0;

                if (rx_buf[sizeof(Dyn_Command_Packet) - 1] == (uint8_t)DYN_END_BYTE)
                    handle_command((const Dyn_Command_Packet*)rx_buf);
                else
                    send_response(DYN_MALFORMED);
            }
        }
    }
}

// ---- Setup ------------------------------------------------------------------
void setup()
{
    pinMode(LED_PIN, OUTPUT);

    dxl.begin(DXL_BAUD);
    dxl.setPortProtocolVersion(DXL_PROTOCOL);

    bool dxl_ok = dxl.ping(ID_LEFT) && dxl.ping(ID_RIGHT);
    if (!dxl_ok) {
        blink_led(5, 80, 80);
    } else {
        dxl.torqueOff(ID_LEFT);
        dxl.setOperatingMode(ID_LEFT, OP_POSITION);
        dxl.torqueOn(ID_LEFT);

        dxl.torqueOff(ID_RIGHT);
        dxl.setOperatingMode(ID_RIGHT, OP_POSITION);
        dxl.torqueOn(ID_RIGHT);

        // Set default profile velocity for smooth tracking.
        set_profile_velocity(ID_LEFT,  DEFAULT_PROFILE_VEL_RPM);
        set_profile_velocity(ID_RIGHT, DEFAULT_PROFILE_VEL_RPM);

        servos_active = true;
    }

    init_sync_write();
    last_cmd_ms = millis();
    espSerial.begin(ESP_BAUD);
    blink_led(3, 400, 200);
}

// ---- Main loop --------------------------------------------------------------
void loop()
{
    poll_esp_serial();

    if (servos_active && (millis() - last_cmd_ms > HEARTBEAT_TIMEOUT_MS)) {
        dxl.torqueOff(ID_LEFT);
        dxl.torqueOff(ID_RIGHT);
        servos_active = false;
    }
}
