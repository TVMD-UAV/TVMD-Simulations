#include "IdleController.h"

IdleController::IdleController(Commander *commander) : Controller(commander)
{
    set_hsl(0, 100, 50);
    set_led_mode(SinWave);
    controller_type = Idle;
    set_led_mode(LED_Mode::PosSinWave);
}

void IdleController::generate_commands()
{
    AgentCommands_t com;
    uint8_t h, s, l;
    generate_led_commands(0, h, s, l);

    com.upper_motor = 0;
    com.lower_motor = 0;
    com.center_servo = 127;
    com.outer_servo = 127;
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