#ifndef CONFIG_H
#define CONFIG_H

#define NUM_AGENT 10

#define UART_BAUDRATE 9600

#define TERMINATE_CHAR (uint8_t) 255
#define BROADCAST_ID 0

#define BUS_RXD 16
#define BUS_TXD 33

// Agent id starting from 1
const uint8_t id_map[NUM_AGENT] = {10, 7, 5, 6, 9, 3, 2, 1, 4, 8};

// The position of agents
const float agent_pos[NUM_AGENT][3] = {
    {-1, 3, 0},  {1, 3, 0},  {0, 2, 0},   {-1, 1, 0}, {1, 1, 0},
    {-1, -1, 0}, {1, -1, 0}, {-2, -2, 0}, {0, -2, 1}, {2, -2, 0}};

#endif