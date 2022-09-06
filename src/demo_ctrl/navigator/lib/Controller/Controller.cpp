#include "Controller.h"

Controller::Controller(Commander *commander)
    : _commander(commander), _update_interval(50), controller_type(None)
{
    set_led_freq(0.1f);
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
    float x, y, z;
    switch (_led_mode) {
    case PosSinWave:
        get_agent_pos(agent_id, x, y, z);
        wh = 127 * (1 + sin(2 * PI * _led_freq * millis() / 1000 + y / 2.0f));
        h = wh;
        s = _s;
        l = _l;
        break;

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

void Controller::get_agent_pos(uint8_t agent_id, float &x, float &y, float &z)
{
    x = agent_pos[agent_id - 1][0];
    y = agent_pos[agent_id - 1][1];
    z = agent_pos[agent_id - 1][2];
}