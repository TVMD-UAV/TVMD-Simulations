#include "Commander.h"

Commander::Commander() : _sending_id(0), mirror_buf(BufferState::Buf1) {}

void Commander::init()
{
    buf = buf1;
    mirror_buf = BufferState::Buf1;

    Serial2.begin(UART_BAUDRATE, SERIAL_8N1, BUS_RXD, BUS_TXD);
}

void Commander::set_agent_commands(const AgentCommands_t *const com)
{
    // Write into mirror
    UartPacket_t *mirror = (mirror_buf == BufferState::Buf1) ? buf1 : buf2;
    if (com->id == BROADCAST_ID) {
        for (uint8_t i = 0; i < NUM_AGENT; i++) {
            memcpy(mirror[i].raw, (void *) com, sizeof(AgentCommands_t));
            mirror[i].data.id = i + 1;
        }
    } else {
        memcpy(mirror[com->id - 1].raw, (void *) com, sizeof(AgentCommands_t));
    }

}

void Commander::packets_swap()
{
    if (mirror_buf == BufferState::Buf1) {
        // buf2
        buf = buf1;
        mirror_buf = BufferState::Buf2;
    } else {
        // buf1
        buf = buf2;
        mirror_buf = BufferState::Buf1;
    }
}

void Commander::send_single_commands(const UartPacket_t *const p)
{
    for (uint8_t j = 0; j < DATA_LEN; j++) {
        uint8_t d = p->raw[j];
        d = (p->raw[j] < 255) ? p->raw[j] : 254;
        Serial2.write(d);
    }
    // Terminator
    Serial2.write(TERMINATE_CHAR);
}

void Commander::send_commands()
{
    packets_swap();
    send_single_commands(&buf[_sending_id]);
    _sending_id = (_sending_id + 1) % NUM_AGENT;
}