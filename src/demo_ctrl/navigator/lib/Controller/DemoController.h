#ifndef DEMO_CONTROLLER_H
#define DEMO_CONTROLLER_H

#include "Controller.h"

class DemoController : public Controller
{
public:
    DemoController(Commander *commander);

    void generate_commands() override;

    void set_motor(Motors motor, uint8_t value) { _motors[motor] = value; };

protected:
    LED_Mode _mode;
    uint8_t _motors[4] = {0};
    uint8_t _h, _s, _l;
};

#endif