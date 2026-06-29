#include "haptic_motor_control.h"

void HapticMotorController::send_packet(Command_Packet packet)
{
    SerialConnection->write((uint8_t*)&packet, sizeof(packet));
}

bool HapticMotorController::send_command(const Command_Packet& packet, Response_Packet* recieved)
{
    send_packet(packet);

    // Fire-and-forget: caller didn't ask for a response.
    if (recieved == NULL)
        return true;

    bool valid_check = false;
    *recieved = read_response(&valid_check, 500);
    return valid_check;
}

Response_Packet HapticMotorController::read_response(bool* valid, unsigned long timeout)
{
    Response_Packet result;
    if (valid != nullptr) *valid = false;

    unsigned long start_time = millis();

    // -----------------------------
    // 1. Find START_BYTE (resync safe)
    // -----------------------------
    bool start_found = false;

    while (millis() - start_time < timeout)
    {
        if (SerialConnection->available())
        {
            char b = SerialConnection->read();

            if (b == START_BYTE)
            {
                result.startByte = START_BYTE;
                start_found = true;
                break;
            }
        }
    }

    if (!start_found)
        return result;

    // -----------------------------
    // 2. Read remaining struct bytes
    // -----------------------------
    size_t remaining = sizeof(Response_Packet) - 1;
    uint8_t* ptr = (uint8_t*)&result;
    size_t index = 1;

    while (remaining > 0 && (millis() - start_time < timeout))
    {
        if (SerialConnection->available())
        {
            ptr[index++] = SerialConnection->read();
            remaining--;
        }
    }

    // timeout or incomplete packet
    if (remaining != 0)
        return result;

    // -----------------------------
    // 3. Validate END_BYTE
    // -----------------------------
    if (result.endByte != END_BYTE)
        return result;

    // -----------------------------
    // 4. Mark valid
    // -----------------------------
    if (valid != nullptr)
        *valid = true;

    return result;
}

bool HapticMotorController::ping_controller(unsigned long wait_time)
{
    // 1. Build ping packet
    Command_Packet packet =
    {
        START_BYTE,
        PING,
        0, 0, 0, 0, 0, 0,
        END_BYTE
    };

    // 2. Send it
    send_packet(packet);

    // 3. Read response
    bool valid = false;
    Response_Packet resp = read_response(&valid, wait_time);

    // 4. Validate response
    if (!valid)
        return false;

    return resp.response == PONG;
}

bool HapticMotorController::set_constant_force(float force, Response_Packet* recieved)
{
    // Torque is a float, but params travel as uint32_t. Bit-cast (not numeric
    // convert) so the fractional part survives the wire.
    uint32_t force_bits;
    memcpy(&force_bits, &force, sizeof(force_bits));

    Command_Packet packet =
    {
        START_BYTE,
        CONSTANT_FORCE,
        force_bits,
        0, 0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}

bool HapticMotorController::relax(Response_Packet* recieved)
{
    // RELAX takes no params; end all intentional force.
    Command_Packet packet =
    {
        START_BYTE,
        RELAX,
        0, 0, 0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}

bool HapticMotorController::vibrate(int frequency, Response_Packet* recieved)
{
    // Vibrate at the given frequency on top of the current base mode.
    Command_Packet packet =
    {
        START_BYTE,
        VIBRATE,
        (uint32_t)frequency,
        0, 0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}

bool HapticMotorController::set_spring_force(float spring_constant, Response_Packet* recieved)
{
    // The spring constant is a float, but params are transmitted as uint32_t.
    // Bit-cast (not numeric convert) so the fractional part survives the wire.
    uint32_t spring_bits;
    memcpy(&spring_bits, &spring_constant, sizeof(spring_bits));

    Command_Packet packet =
    {
        START_BYTE,
        SPRING_FORCE,
        spring_bits,
        0, 0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}