#include "JoystickController.h"

JoystickController::JoystickController(Commander *commander):
Controller(commander){
    set_hsl(0, 50, 100);
}

void JoystickController::generate_commands(){
    AgentCommands_t com;
    // calculate each drone's motor thrust and servo angle
    // using joystick's data _x, _y
    com = {
        .id = 0,
        .upper_motor = 0,
        .lower_motor = 0,
        .center_servo = 0,
        .outer_servo = 0,
        .h=0, .s=50, .l=100
    };
    set_agent_commands(&com);
    _commander->update_packets();
}