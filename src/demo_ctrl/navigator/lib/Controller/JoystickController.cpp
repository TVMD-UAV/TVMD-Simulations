#include "JoystickController.h"

JoystickController::JoystickController(Commander *commander)
    : Controller(commander)
{
    set_hsl(0, 50, 100);
    controller_type = JoyStick;
}

void JoystickController::generate_commands()
{
    AgentCommands_t com;
    // calculate each drone's motor thrust and servo angle
    // using joystick's data _x, _y

    if (_flight_mode == FlightMode::Hover) {
        com = {.id = 0,
               .upper_motor = 0,
               .lower_motor = 0,
               .center_servo = 0,
               .outer_servo = 0,
               .h = _h,
               .s = _s,
               .l = _l};

        double eta = atan2((double) -_inputs[1], (double) _inputs[2]);
        double xi = atan2(cos(eta) * (double) _inputs[0], (double) _inputs[2]);
        for (int i = 0; i < 6; i++) {
            Serial.print(_inputs[i]);
            Serial.print(", ");
        }

        Serial.print(" | ");
        Serial.print(eta);
        Serial.print(", ");
        Serial.println(xi);

        for (uint8_t i = 1; i <= NUM_AGENT; i++) {
            com.outer_servo = (uint8_t)(eta * 10 + 127);
            com.center_servo = (uint8_t)(xi * 10 + 127);
            _commander->set_agent_commands(&com);
        }
    }
}

void JoystickController::parse_input(int *inputs_arr, int len)
{
    if (len == 1) {
        _flight_mode = (FlightMode) inputs_arr[0];
    } else if (len == 6) {
        // Set inputs
        memcpy(_inputs, inputs_arr, sizeof(_inputs));
        _inputs[2] += 50;
    }
}