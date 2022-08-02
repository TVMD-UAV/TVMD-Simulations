#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include "Controller.h"

#define FixWingModeServoAngle 250
#define HoverModeServoAngle 127

#define HoverMotorSpeed 127
#define FixWingMotorSpeed 230
#define TransitionMotorSpeed 150

#define TimeForTransition 5000  // in milliseconds

enum FlightMode {
    Hover = 0,
    FixedWing,
    TransitionToHover,
    TransitionToFixedWing
};
class JoystickController : public Controller
{
public:
    enum InputState { Buf1, Buf2 };
    InputState mirror_input;
    JoystickController(Commander *commander);

    void generate_commands() override;

    void parse_input(int *inputs_arr, int len) override;

protected:
    uint8_t _x, _y;

    FlightMode _flight_mode;

    // 0: hover mode
    // 1: fixed wing
    float _transition_state;

    int _inputs1[6];
    int _inputs2[6];

    float calc_transition_servo_angle();

    uint32_t _transistion_start_time;
};

#endif