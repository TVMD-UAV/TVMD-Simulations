#ifndef ISRLED_H
#define ISRLED_H

#include "Arduino.h"

#define LED_TYPE uint8_t
#define LED_LEVEL 15

class IsrLed {
public:
  IsrLed();

  void init();

  /* Set the leds via HSL color space.
   * @Params:
   *      H: Hue, 0~360
   *      S: Saturation, 0~1
   *      V: Value, 0~1, (Set to 1 for white, 0 for black)
   */
  void set_hsl(int H, float S, float L);

  static void update();

protected:
  static volatile LED_TYPE _leds[3];

  float hue_to_rgb(float v1, float v2, float vH);

  void ISR_enable();

  void ISR_disable();
};
#endif