#ifndef IDLE_CONTROLLER_H
#define IDLE_CONTROLLER_H

#include "Controller.h"

class IdleController:public Controller{
    public:
    IdleController(Commander *commander);

    void generate_commands() override;

    enum LED_Mode {Regular, SinWave};

    void set_mode(LED_Mode new_mode) {_mode = new_mode;};

    /*
     @Params
        h: 0~360
        s: 0~1
        l: 0~1
     */
    void set_hsl(float h, float s, float l) {
        _h = (uint8_t) (h * 254 / 360);
        _s = (uint8_t) (s * 254); 
        _l = (uint8_t) (l * 254);};

    void set_update_freq(float update_freq) {_update_freq = update_freq;};

    protected:
    LED_Mode _mode;
    uint8_t _h, _s, _l;
    float _update_freq;
};

#endif