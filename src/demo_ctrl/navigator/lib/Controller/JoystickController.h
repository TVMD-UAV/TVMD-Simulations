#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include "Controller.h"

class JoystickController : public Controller
{
public:
    JoystickController(Commander *commander);

    void generate_commands() override;

    void set_direction(int x, int y) { _x = x, _y = y; };

protected:
    uint8_t _x, _y;
};

#endif