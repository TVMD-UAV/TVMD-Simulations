#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Commander.h"

class Controller
{
public:
    enum LED_Mode { Regular, SinWave };
    enum Motors { Upper, Lower, Center, Outer };

    /********************* Global functions *********************/
    Controller(Commander *commander);

    // Set control loop update frequency, default: 50 Hz
    void set_update_freq(float update_freq)
    {
        _update_interval = 1000.0f / update_freq;
    };

    // This function must be called in the main loop
    void update();

    // User defined input
    /* This function provides an opportunity for the users to set
       customized parameters which are not declared in Controller. */
    virtual void parse_input(int *inputs_arr, int len){};

    // User defined control scheme
    /* Responsibility:
     *    - Generating commands (including leds and motors) according to the
     * configuration
     *    - Call set_agent_commands() to set commands of each agents
     */
    virtual void generate_commands() = 0;

    /*********************** LED control ***********************/
    // Set LED modes
    void set_led_mode(LED_Mode new_mode) { _led_mode = new_mode; };

    // Set HSL value
    /* Range:
     *    - H: 0~360 (int)
     *    - S: 0~100 (int)
     *    - L: 0~100 (int)
     */
    void set_hsl(int h, int s, int l)
    {
        _h = (uint8_t)(h * 254.0 / 360.0);
        _s = (uint8_t)(s * 254.0 / 100.0);
        _l = (uint8_t)(l * 254.0 / 100.0);
    };

    void set_led_freq(float freq) { _led_freq = freq; };

    virtual void generate_led_commands(uint8_t agent_id,
                                       uint8_t &h,
                                       uint8_t &s,
                                       uint8_t &l);

    virtual void set_motor(Motors, uint8_t){};

    // A helper to set single agent
    void set_agent_commands(const AgentCommands_t *const com);

protected:
    Commander *_commander;
    LED_Mode _led_mode;
    uint32_t _update_interval;

    uint8_t _h, _s, _l;
    float _led_freq;
};

#endif