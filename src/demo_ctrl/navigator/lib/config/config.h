#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define N_PACKETS 10
#define DATA_LEN 8

typedef struct _PacketData_t{
    uint8_t id;
    uint8_t upper_motor, bottom_motor;
    uint8_t center_servo, outer_servo;
    uint8_t r, g, b;
} PacketData_t;

enum Modes {None=0, Idle, Demo, Joystick, Fixwing};

#endif