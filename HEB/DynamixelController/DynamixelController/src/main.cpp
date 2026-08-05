#include <Arduino.h>
#include <DynamixelShield.h>
#include "dynamixel_protocol.h"
#include "Slave_Side.h"

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

// ---- ESP32 SPI link ----------------------------------------------------------
// Hardware SPI is fixed by the AVR: SS=D10, MOSI=D11, MISO=D12, SCK=D13 --
// repurposed from the old SoftwareSerial(10,12) UART link (D10/D12) plus two
// previously-idle pins (D11/D13). Hardware Serial (pins 0/1) is owned by
// DynamixelShield; pin 2 is DXL_DIR_PIN -- neither is touched here.
#define DATA_READY_PIN 9
Slave_Controller slave(DATA_READY_PIN);

// ---- Heartbeat safety -------------------------------------------------------
#define HEARTBEAT_TIMEOUT_MS 500UL

static unsigned long last_cmd_ms   = 0;
static bool          servos_active = false;

// ---- Goal / settle state -----------------------------------------------------
static float         goal_left_deg     = 0.0f;
static float         goal_right_deg    = 0.0f;
static bool          settled_notified  = true; // true = nothing pending to notify

static unsigned long last_position_poll_ms = 0;
#define POSITION_POLL_INTERVAL_MS 20UL // throttles MOVING/position reads on the
                                        // half-duplex Dynamixel UART bus

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

// ---- Command handler --------------------------------------------------------
static void handle_command(const local_packet& lp)
{
    switch (lp.command)
    {
        case CMD_DXL_MOVE:
        {
            float left_deg, right_deg, left_spd, right_spd;
            memcpy(&left_deg,  &lp.data1, sizeof(left_deg));
            memcpy(&right_deg, &lp.data2, sizeof(right_deg));
            memcpy(&left_spd,  &lp.data3, sizeof(left_spd));
            memcpy(&right_spd, &lp.data4, sizeof(right_spd));

            if (!servos_active) {
                dxl.torqueOn(ID_LEFT);
                dxl.torqueOn(ID_RIGHT);
                servos_active = true;
            }

            // Override profile velocity only when explicitly requested.
            set_profile_velocity(ID_LEFT,  left_spd);
            set_profile_velocity(ID_RIGHT, right_spd);

            goal_left_deg  = left_deg;
            goal_right_deg = right_deg;
            sync_move(left_deg, right_deg); // non-blocking; retargets in-flight motion
            settled_notified = false;        // re-arm: must notify once THIS goal settles
            break;
        }

        case CMD_DXL_STOP:
            dxl.torqueOff(ID_LEFT);
            dxl.torqueOff(ID_RIGHT);
            servos_active    = false;
            settled_notified = true; // suppress a stale notify for an abandoned goal
            break;

        case CMD_DXL_PING:
            // No action needed - the fast ACK already confirms the link.
            break;

        default:
            break;
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
    blink_led(3, 400, 200);

    // Must be last: claims D13 for hardware SPI SCK. D13 is also the onboard
    // LED, so no blink_led()/digitalWrite(LED_PIN,...) may run after this --
    // driving it as a GPIO once SPI slave mode is enabled would contend with
    // the SPI master's SCK drive.
    slave.begin();
}

// ---- Main loop --------------------------------------------------------------
void loop()
{
    local_packet lp;
    if (slave.get_command(&lp) && lp.command != 0)
    {
        last_cmd_ms = millis(); // refreshed on every successfully-received command
        handle_command(lp);
    }

    // Heartbeat safety timeout - unchanged behavior from the old UART link.
    if (servos_active && (millis() - last_cmd_ms > HEARTBEAT_TIMEOUT_MS)) {
        dxl.torqueOff(ID_LEFT);
        dxl.torqueOff(ID_RIGHT);
        servos_active = false;
    }

    // Settle detection + one-shot telemetry notify.
    if (servos_active && !settled_notified &&
        (millis() - last_position_poll_ms >= POSITION_POLL_INTERVAL_MS))
    {
        last_position_poll_ms = millis();

        bool left_moving  = dxl.readControlTableItem(ControlTableItem::MOVING, ID_LEFT)  != 0;
        bool right_moving = dxl.readControlTableItem(ControlTableItem::MOVING, ID_RIGHT) != 0;

        if (!left_moving && !right_moving)
        {
            float actual_left  = dxl.getPresentPosition(ID_LEFT,  UNIT_DEGREE);
            float actual_right = dxl.getPresentPosition(ID_RIGHT, UNIT_DEGREE);

            local_packet settle_lp;
            settle_lp.command = CMD_DXL_SETTLED;
            memcpy(&settle_lp.data1, &goal_left_deg,   sizeof(float));
            memcpy(&settle_lp.data2, &goal_right_deg,  sizeof(float));
            memcpy(&settle_lp.data3, &actual_left,     sizeof(float));
            memcpy(&settle_lp.data4, &actual_right,    sizeof(float));

            slave.notify_data(&settle_lp);
            settled_notified = true;
        }
    }
}
