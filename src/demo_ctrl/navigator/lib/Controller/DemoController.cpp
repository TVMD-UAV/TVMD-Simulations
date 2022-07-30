#include "DemoController.h"

DemoController::DemoController(Commander *commander) : Controller(commander)
{
    set_hsl(0, 50, 100);
}

void DemoController::generate_commands()
{
    AgentCommands_t com;
    switch (_mode) {
    case Regular:
        com = {.id = 0,
               .upper_motor = _motors[0],
               .lower_motor = _motors[1],
               .center_servo = _motors[2],
               .outer_servo = _motors[3],
               .h = _h,
               .s = _s,
               .l = _l};
        set_agent_commands(&com);
        break;

    case SinWave:
        uint8_t h = 127 * (1 + sin(2 * PI * _update_freq * millis() / 1000));
        com = {.id = 0,
               .upper_motor = 0,
               .lower_motor = 0,
               .center_servo = 0,
               .outer_servo = 0,
               .h = h,
               .s = _s,
               .l = _l};
        set_agent_commands(&com);
        break;

        // default:

        //    break;
    }
    _commander->update_packets();
}