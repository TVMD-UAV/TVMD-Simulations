#include "DemoController.h"

DemoController::DemoController(Commander *commander) : Controller(commander)
{
    set_hsl(0, 50, 100);
    set_led_mode(SinWave);
}

void DemoController::generate_commands()
{
    AgentCommands_t com;

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
    _commander->set_agent_commands(&com);
}