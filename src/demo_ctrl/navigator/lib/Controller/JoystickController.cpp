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

    // Check transition state

    // calculate each drone's motor thrust and servo angle
    if (_flight_mode == FlightMode::Hover) {
        com = {.id = 0,
               .upper_motor = HoverMotorSpeed,
               .lower_motor = HoverMotorSpeed,
               .center_servo = 127,
               .outer_servo = 127,
               .h = _h,
               .s = _s,
               .l = _l};

        int *_inputs = (mirror_input == InputState::Buf1) ? _inputs2 : _inputs1;

        double eta = -atan2((double) -_inputs[1], (double) _inputs[2]);
        double xi = -atan2(cos(eta) * (double) _inputs[0], (double) _inputs[2]);
        for (int i = 0; i < 6; i++) {
            Serial.print(_inputs[i]);
            Serial.print(", ");
        }

        int motor_speed = HoverMotorSpeed + _inputs[2];
        motor_speed =
            (motor_speed > 254) ? 254 : ((motor_speed <= 0) ? 0 : motor_speed);

        Serial.print(" | ");
        Serial.print(eta);
        Serial.print(", ");
        Serial.println(xi);

        for (uint8_t i = 1; i <= NUM_AGENT; i++) {
            com.upper_motor = (uint8_t) motor_speed;
            com.lower_motor = (uint8_t) motor_speed;
            com.outer_servo = (uint8_t)(eta * 10 + 127);
            com.center_servo = (uint8_t)(xi * 10 + 127);
            _commander->set_agent_commands(&com);
        }
        /*if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 5) == pdTRUE) {
            xSemaphoreGive(xSerialSemaphore);  // Now free or "Give" the Serial
            // Port for others.

        }*/
    } else if (_flight_mode == FlightMode::FixedWing) {
        com = {.id = 0,
               .upper_motor = 40,
               .lower_motor = 40,
               .center_servo = 127,
               .outer_servo = FixWingModeServoAngle,
               .h = _h,
               .s = _s,
               .l = _l};
        for (uint8_t i = 1; i <= NUM_AGENT; i++) {
            if (id_map[i - 1] == 7 || id_map[i - 1] == 4 ||
                id_map[i - 1] == 8) {
                com.id = id_map[i - 1];
                com.upper_motor = FixWingMotorSpeed;
                com.lower_motor = FixWingMotorSpeed;
            }
            _commander->set_agent_commands(&com);
        }
    } else {
        com.id = 0;
        com.upper_motor = TransitionMotorSpeed;
        com.lower_motor = TransitionMotorSpeed;
        com.center_servo = 127;
        com.outer_servo = (uint8_t) calc_transition_servo_angle();
        Serial.print("Transition Servo angle: ");
        Serial.print(com.outer_servo);

        Serial.print(", transition: ");
        Serial.println(_transition_state);

        _transition_state =
            ((float) (millis() - _transistion_start_time)) / TimeForTransition;

        if (_transition_state >= 1.0f) {
            _flight_mode = (_flight_mode == FlightMode::TransitionToFixedWing)
                               ? FlightMode::FixedWing
                               : FlightMode::Hover;
            _transition_state = 0;
        }
        _commander->set_agent_commands(&com);
    }
}

void JoystickController::parse_input(int *inputs_arr, int len)
{
    int *mirror = (mirror_input == InputState::Buf1) ? _inputs1 : _inputs2;
    if (len == 1) {
        if (_flight_mode == FlightMode::Hover &&
            inputs_arr[0] == FlightMode::FixedWing) {
            _flight_mode = FlightMode::TransitionToFixedWing;
            _transistion_start_time = millis();
            _transition_state = 0;
        } else if (_flight_mode == FlightMode::FixedWing &&
                   inputs_arr[0] == FlightMode::Hover) {
            _flight_mode = FlightMode::TransitionToHover;
            _transistion_start_time = millis();
            _transition_state = 0;
        } else if (_flight_mode != FlightMode::TransitionToHover &&
                   _flight_mode != FlightMode::TransitionToFixedWing) {
            _flight_mode = (FlightMode) inputs_arr[0];
        }
    } else if (len == 6) {
        // Set inputs
        memcpy(mirror, inputs_arr, sizeof(_inputs1));
        mirror[2] += 50;

        if (mirror_input == InputState::Buf1)
            mirror_input = InputState::Buf2;
        else
            mirror_input = InputState::Buf1;
    }
}

float JoystickController::calc_transition_servo_angle()
{
    float t = (_flight_mode == FlightMode::TransitionToFixedWing)
                  ? _transition_state
                  : (1.0f - _transition_state);
    float angle =
        HoverModeServoAngle + t * (FixWingModeServoAngle - HoverModeServoAngle);
    angle = (angle > FixWingModeServoAngle) ? FixWingModeServoAngle : angle;
    return angle;
};