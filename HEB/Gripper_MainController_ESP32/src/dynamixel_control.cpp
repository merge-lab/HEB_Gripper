#include "dynamixel_control.h"

DynamixelController::DynamixelController(int mosi_pin, int miso_pin, int sck_pin,
                                          int ss_pin, int data_ready_pin)
    : _ss_pin(ss_pin), _dr_pin(data_ready_pin),
      _master(mosi_pin, miso_pin, sck_pin, /*num_slaves=*/1, &_ss_pin, &_dr_pin)
{}

void DynamixelController::begin() { _master.begin(); }

void DynamixelController::update()
{
    _master.process();
    local_packet lp;
    while (_master.get_data(SLAVE_IDX, &lp))
    {
        if (lp.command == CMD_DXL_SETTLED)
        {
            memcpy(&_latest.goal_left_deg,    &lp.data1, sizeof(float));
            memcpy(&_latest.goal_right_deg,   &lp.data2, sizeof(float));
            memcpy(&_latest.actual_left_deg,  &lp.data3, sizeof(float));
            memcpy(&_latest.actual_right_deg, &lp.data4, sizeof(float));
            _has_new_telemetry = true;
        }
        // else: ignore (command==0, or a stray/unexpected value) - defensive only.
    }
}

bool DynamixelController::send_command(uint8_t command, float d1, float d2, float d3, float d4)
{
    local_packet lp;
    lp.command = command;
    memcpy(&lp.data1, &d1, sizeof(float));
    memcpy(&lp.data2, &d2, sizeof(float));
    memcpy(&lp.data3, &d3, sizeof(float));
    memcpy(&lp.data4, &d4, sizeof(float));

    return _master.send_data(&lp, SLAVE_IDX);
}

bool DynamixelController::move(float left_deg, float right_deg,
                               float left_speed_rpm, float right_speed_rpm)
{
    return send_command(CMD_DXL_MOVE, left_deg, right_deg, left_speed_rpm, right_speed_rpm);
}

bool DynamixelController::stop()
{
    return send_command(CMD_DXL_STOP, 0, 0, 0, 0);
}

bool DynamixelController::ping()
{
    return send_command(CMD_DXL_PING, 0, 0, 0, 0);
}

bool DynamixelController::get_latest_position(Telemetry* out)
{
    *out = _latest;
    bool is_new = _has_new_telemetry;
    _has_new_telemetry = false;
    return is_new;
}
