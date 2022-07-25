#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Commander.h"

class Controller{
    public:
    Controller(Commander *commander);

    void set_agent_commands(const AgentCommands_t * const com);

    virtual void generate_commands() = 0;

    protected:
    Commander *_commander;
};

#endif