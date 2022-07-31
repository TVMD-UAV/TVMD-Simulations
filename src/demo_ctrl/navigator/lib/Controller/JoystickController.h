#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include "Controller.h"

enum FlightMode { Hover = 0, FixedWing };
class JoystickController : public Controller
{
public:
    JoystickController(Commander *commander);

    void generate_commands() override;

    void parse_input(int *inputs_arr, int len) override;

protected:
    uint8_t _x, _y;

    FlightMode _flight_mode;

    int _inputs[6];
};

#endif