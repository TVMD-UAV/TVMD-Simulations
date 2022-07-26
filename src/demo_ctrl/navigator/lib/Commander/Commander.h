#ifndef COMMANDER_H
#define COMMANDER_H

#include <Arduino.h>
#include "config.h"

#define DATA_LEN 8

struct AgentCommands_t{
    uint8_t id;
    uint8_t upper_motor, lower_motor;
    uint8_t center_servo, outer_servo;
    uint8_t h, s, l;
};

union UartPacket_t{
    uint8_t raw[DATA_LEN];
    AgentCommands_t data;
};

class Commander{
    public:
    Commander();
    void init();

    UartPacket_t buf1[NUM_AGENT];
    UartPacket_t buf2[NUM_AGENT];

    // For double buffered packets
    UartPacket_t *buf;

    void set_agent_commands(const AgentCommands_t * const com);

    void update_packets();

    void send_commands();

    protected:

    // Indicating which buffer is used, false means buf1.
    bool buffer_state;
};



#endif