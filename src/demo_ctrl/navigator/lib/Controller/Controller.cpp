#include "Controller.h"

Controller::Controller(Commander *commander)
    : _commander(commander), _update_interval(20)
{
    set_led_freq(1.0f);
    set_led_mode(Regular);
}

void Controller::set_agent_commands(const AgentCommands_t *const commands)
{
    _commander->set_agent_commands(commands);
}

void Controller::update()
{
    static auto last_update_time = millis();
    if (millis() - last_update_time > _update_interval) {
        last_update_time = millis();
        generate_commands();
        _commander->send_commands();
    }
}

void Controller::generate_led_commands(uint8_t agent_id,
                                       uint8_t &h,
                                       uint8_t &s,
                                       uint8_t &l)
{
    uint8_t wh;
    switch (_led_mode) {
    case SinWave:
        wh = 127 * (1 + sin(2 * PI * _led_freq * millis() / 1000));
        h = wh;
        s = _s;
        l = _l;
        break;

    case Regular:
    default:
        h = _h;
        s = _s;
        l = _l;
        break;
    }
}