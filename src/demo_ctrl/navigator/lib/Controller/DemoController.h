#ifndef DEMO_CONTROLLER_H
#define DEMO_CONTROLLER_H

#include "Controller.h"

enum MotorMode { Fixed = 0, SineWave };

enum MotorType { Upper = 0, Lower, Center, Outer };

class DemoController : public Controller
{
public:
    DemoController(Commander *commander);

    void generate_commands() override;

    void set_motor(MotorType motor, uint8_t value) { _motors[motor] = value; };

    void parse_input(int *inputs_arr, int len) override;

protected:
    MotorMode _motor_mode;
    uint8_t _motors[4] = {0};
};

#endif