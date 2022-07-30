#ifndef IDLE_CONTROLLER_H
#define IDLE_CONTROLLER_H

#include "Controller.h"

class IdleController : public Controller
{
public:
    IdleController(Commander *commander);

    void generate_commands() override;
};

#endif