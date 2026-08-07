#include <Arduino.h>
#include <DynamixelShield.h>
#include <Servo_Control.h>
#include "dynamixel_protocol.h"
#include "Slave_Side.h"


// ---- Dynamixel config -------------------------------------------------------
DynamixelShield dxl;
const float     DXL_PROTOCOL  = 2.0;
const long      DXL_BAUD      = 1000000;

// Indices 0/1 below double as both the raw Dynamixel hardware IDs (this
// project's servos happen to be IDs 0/1) and the array positions used for
// every Servo_Controller call (positions/speeds/currents). If the hardware
// IDs ever change, SERVO_IDS still must stay index-ordered left, then right.
const uint8_t   ID_LEFT       = 0;
const uint8_t   ID_RIGHT      = 1;
const int       SERVO_IDS[]   = { ID_LEFT, ID_RIGHT };
const size_t    NUM_SERVOS    = 2;

// Default profile velocity for smooth tracking. Explicit speed in a move
// command overrides this. 0 = max speed (no limit).
#define DEFAULT_PROFILE_VEL_RPM  0.0f

// Goal Position register for Protocol 2.0 servos (XM, XH, XL430, MX 2.0).
// 4096 counts per revolution; raw = degrees * 4096 / 360.
#define GOAL_POS_ADDR  116
#define GOAL_POS_LEN   4

static Servo_Controller* servo_controller = nullptr;

// ---- ESP32 serial link ------------------------------------------------------
// Hardware Serial (pins 0/1) is owned by DynamixelShield.
// Serial2 used by mega (pins 16 and 17)
const long     ESP_BAUD = 250000;

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

static inline int32_t deg_to_raw(float deg)
{
    return (int32_t)(deg * (4096.0f / 360.0f));
}

// Move both servos simultaneously via a single Sync Write packet.
static bool sync_move(float left_deg, float right_deg)
{
    int positions[2] = { deg_to_raw(left_deg), deg_to_raw(right_deg) };
    return servo_controller->set_postions(positions, 2);
}

// ---- Response sender --------------------------------------------------------
static void send_response(Dyn_Response resp_code)
{
    Dyn_Response_Packet pkt;
    pkt.startByte = DYN_START_BYTE;
    pkt.response  = (uint8_t)resp_code;

    float currents[2];
    servo_controller->get_current_draw(currents, 2);

    memcpy(&pkt.left_motor_current,  &currents[0], sizeof(pkt.left_motor_current));
    memcpy(&pkt.right_motor_current, &currents[1], sizeof(pkt.right_motor_current));

    pkt.endByte = DYN_END_BYTE;
    Serial2.write((const uint8_t*)&pkt, sizeof(pkt));
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
                servo_controller->torque_on();
                servos_active = true;
            }

            // Override profile velocity only when explicitly requested.
            float speeds[2] = { left_spd, right_spd };
            servo_controller->set_speed(speeds, 2);

            bool ok = sync_move(left_deg, right_deg);
            send_response(ok ? DYN_ACK : DYN_FAILURE);
            break;
        }

        case DYN_MOVE_SERVO_FAST:
        {
            float left_deg, right_deg;
            memcpy(&left_deg,  &cmd->left_degrees,  sizeof(left_deg));
            memcpy(&right_deg, &cmd->right_degrees, sizeof(right_deg));

            if (!servos_active) {
                servo_controller->torque_on();
                servos_active = true;
            }

            sync_move(left_deg, right_deg);
            break;
        }

        case DYN_STOP:
            servo_controller->torque_off();
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
    while (Serial2.available())
    {
        uint8_t b = Serial2.read();

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

    // Constructed here (not as a global object) because Servo_Controller's
    // constructor does real hardware I/O (dxl.begin, register writes), which
    // must happen after Arduino's own init() has run, not during static
    // initialization before setup().
    servo_controller = new Servo_Controller(dxl, SERVO_IDS, NUM_SERVOS, DXL_PROTOCOL, DXL_BAUD,
        GOAL_POS_ADDR, GOAL_POS_LEN, OP_POSITION, DEFAULT_PROFILE_VEL_RPM);

    bool dxl_ok = servo_controller->ping();
    if (!dxl_ok) {
        blink_led(5, 80, 80);
    } else {
        servos_active = true;
    }

    last_cmd_ms = millis();
    Serial2.begin(ESP_BAUD);
    blink_led(3, 400, 200);
}

// ---- Main loop --------------------------------------------------------------
void loop()
{
    poll_esp_serial();

    if (servos_active && (millis() - last_cmd_ms > HEARTBEAT_TIMEOUT_MS)) {
        servo_controller->torque_off();
        servos_active = false;
    }
}
