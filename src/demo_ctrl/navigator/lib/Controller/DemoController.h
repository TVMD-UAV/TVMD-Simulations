#ifndef DEMO_CONTROLLER_H
#define DEMO_CONTROLLER_H

#include "Controller.h"

class DemoController:public Controller{
    public:
    DemoController(Commander *commander);

    void generate_commands() override;

    void set_mode(LED_Mode new_mode) {_mode = new_mode;};

    void set_motor(Motors motor, uint8_t value) {_motors[motor] = value;};

    /*
     @Params
        h: 0~360
        s: 0~1
        l: 0~1
     */
    void set_hsl(int h, int s, int l) {
        _h = (uint8_t) (h * 254.0 / 360.0);
        _s = (uint8_t) (s * 254.0 / 100.0); 
        _l = (uint8_t) (l * 254.0 / 100.0);};

    void set_update_freq(float update_freq) {_update_freq = update_freq;};

    protected:
    LED_Mode _mode;
    uint8_t _motors[4] = {0};
    uint8_t _h, _s, _l;
    float _update_freq;
};

#endif