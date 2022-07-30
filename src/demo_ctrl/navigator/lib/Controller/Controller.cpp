#include "Controller.h"

Controller::Controller(Commander *commander) : _commander(commander) {}

void Controller::set_agent_commands(const AgentCommands_t *const commands)
{
    _commander->set_agent_commands(commands);
}