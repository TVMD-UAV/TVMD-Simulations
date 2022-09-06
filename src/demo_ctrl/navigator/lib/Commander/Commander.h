#ifndef COMMANDER_H
#define COMMANDER_H

#include <Arduino.h>
#include "config.h"

#define DATA_LEN 8

struct AgentCommands_t {
    uint8_t id;
    uint8_t upper_motor, lower_motor;
    uint8_t center_servo, outer_servo;
    uint8_t h, s, l;
};

union UartPacket_t {
    uint8_t raw[DATA_LEN];
    AgentCommands_t data;
};

class Commander
{
public:
    Commander();
    void init();

    // Set agent command by ID, it will broadcast to all the agents
    // if a BROADCAST_ID has been set.
    void set_agent_commands(const AgentCommands_t *const com);

    // Send commands to single agent
    void send_single_commands(const UartPacket_t *const p);

    // Send commands to all agents
    void send_commands();

protected:
    int _sending_id;

    // Indicating which buffer is used, false means buf1.
    enum BufferState { Buf1, Buf2 };
    BufferState mirror_buf;

    UartPacket_t buf1[NUM_AGENT];
    UartPacket_t buf2[NUM_AGENT];

    // For double buffered packets
    UartPacket_t *buf;


    // Swapping the double mirrored buffer, this will be called automatically by
    // send_commands()
    void packets_swap();
};



#endif