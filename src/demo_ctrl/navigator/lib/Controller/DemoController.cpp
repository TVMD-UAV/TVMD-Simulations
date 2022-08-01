#include "DemoController.h"

DemoController::DemoController(Commander *commander) : Controller(commander)
{
    set_hsl(0, 50, 100);
    set_led_mode(LED_Mode::PosSinWave);
    controller_type = Demo;
    _motors[0] = 0;
    _motors[1] = 0;
    _motors[2] = 127;
    _motors[3] = 127;
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
        _commander->set_agent_commands(&com);
    } else if (_motor_mode == MotorMode::SineWave) {
        // Motors
        uint8_t mx = 200 * (sin((float) millis() / 1000) + 1);
        uint8_t my = 200 * (cos((float) millis() / 1000) + 1);

        uint8_t sx = 20 * (sin((float) millis() / 1000)) + 127;
        uint8_t sy = 20 * (cos((float) millis() / 1000)) + 127;

        // LEDs
        uint8_t h, s, l;
        com.upper_motor = mx;
        com.upper_motor = my;
        com.center_servo = sy;
        com.outer_servo = sx;

        for (uint8_t i = 1; i <= NUM_AGENT; i++) {
            generate_led_commands(i, h, s, l);
            com.id = id_map[i - 1];
            com.h = h;
            com.s = s;
            com.l = l;
            // Using broadcast mode by setting ID=0
            _commander->set_agent_commands(&com);
        }
    }
}

void DemoController::parse_input(int *inputs_arr, int len)
{
    if (len == 1) {
        _motor_mode = (MotorMode) inputs_arr[0];
        if (_motor_mode == MotorMode::Fixed)

            set_led_mode(LED_Mode::Regular);
        else
            set_led_mode(LED_Mode::SinWave);
    } else if (len == 2) {
        MotorType which_motor = (MotorType) inputs_arr[0];
        if (which_motor >= 0 && which_motor <= 3)
            _motors[which_motor] = inputs_arr[1];
    }
}