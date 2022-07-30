#include "IdleController.h"

IdleController::IdleController(Commander *commander) : Controller(commander)
{
    set_hsl(0, 50, 100);
    set_led_mode(Regular);
}

void IdleController::generate_commands()
{
    AgentCommands_t com;
    uint8_t h, s, l;
    generate_led_commands(0, h, s, l);

    // Using broadcast mode by setting ID=0
    com = {.id = 0,
           .upper_motor = 0,
           .lower_motor = 0,
           .center_servo = 127,
           .outer_servo = 127,
           .h = h,
           .s = s,
           .l = l};
    _commander->set_agent_commands(&com);
}