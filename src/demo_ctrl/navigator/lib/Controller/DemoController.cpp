#include "DemoController.h"

DemoController::DemoController(Commander *commander) : Controller(commander)
{
    set_hsl(0, 50, 100);
    set_led_mode(Regular);
    controller_type = Demo;
}

void DemoController::generate_commands()
{
    AgentCommands_t com;

    if (_motor_mode == MotorMode::Fixed) {
        com = {.id = 0,
               .upper_motor = _motors[0],
               .lower_motor = _motors[1],
               .center_servo = _motors[2],
               .outer_servo = _motors[3],
               .h = _h,
               .s = _s,
               .l = _l};
    } else if (_motor_mode == MotorMode::SineWave) {
    // Motors
    uint8_t mx = 200 * (sin((float) millis() / 1000) + 1);
    uint8_t my = 200 * (cos((float) millis() / 1000) + 1);

    uint8_t sx = 20 * (sin((float) millis() / 1000)) + 127;
    uint8_t sy = 20 * (cos((float) millis() / 1000)) + 127;

    // LEDs
    uint8_t h, s, l;
    generate_led_commands(0, h, s, l);

    // Using broadcast mode by setting ID=0
    com = {.id = 0,
           .upper_motor = mx,
           .lower_motor = my,
           .center_servo = sy,
           .outer_servo = sx,
           .h = h,
           .s = s,
           .l = l};
    }
    _commander->set_agent_commands(&com);
}

void DemoController::generate_led_commands(uint8_t agent_id,
                                           uint8_t &h,
                                           uint8_t &s,
                                           uint8_t &l)
{
    float agent_x, agent_y, agent_z;
    get_agent_pos(agent_id, agent_x, agent_y, agent_z);
    uint8_t wh =
        127 * (1 + sin(2 * PI * _led_freq * millis() / 1000 + agent_y / 1));
    h = wh;
    s = _s;
    l = _l;
}

void DemoController::parse_input(int *inputs_arr, int len)
{
    if (len == 1) {
        _motor_mode = (MotorMode) inputs_arr[0];
    } else if (len == 2) {
        MotorType which_motor = (MotorType) inputs_arr[0];
        if (which_motor >= 0 && which_motor <= 3)
            _motors[which_motor] = inputs_arr[1];
    }
}