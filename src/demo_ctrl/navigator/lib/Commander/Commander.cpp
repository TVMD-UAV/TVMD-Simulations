#include "Commander.h"

Commander::Commander() {}

void Commander::init()
{
    Serial2.begin(UART_BAUDRATE, SERIAL_8N1, BUS_RXD, BUS_TXD);
}

void Commander::set_agent_commands(const AgentCommands_t *const com)
{
    if (com->id == BROADCAST_ID) {
        for (uint8_t i = 0; i < NUM_AGENT; i++) {
            memcpy(buf[i].raw, (void *) com, sizeof(AgentCommands_t));
            buf[i].data.id = i + 1;
        }
    } else {
        memcpy(buf[com->id].raw, (void *) com, sizeof(AgentCommands_t));
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
    for (uint8_t i = 0; i < NUM_AGENT; i++) {
        send_single_commands(&buf[i]);
    }
}