#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Commander.h"

class Controller
{
public:
    Controller(Commander *commander);

    void set_agent_commands(const AgentCommands_t *const com);

    virtual void generate_commands() = 0;

    enum LED_Mode { Regular, SinWave };
    enum Motors { Upper, Lower, Center, Outer };

    virtual void set_mode(LED_Mode){};
    virtual void set_motor(Motors, uint8_t){};
    virtual void set_hsl(int, int, int){};
    virtual void set_direction(int, int){};

protected:
    Commander *_commander;
};

#endif