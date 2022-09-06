#include "IsrLed.h"

volatile LED_TYPE IsrLed::_leds[3];

IsrLed::IsrLed() {}

void IsrLed::init() {
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  ISR_enable();
}

void IsrLed::ISR_enable() {
  // atmega 328p timer register setting
  // https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328

  ISR_disable();
  // 10k = 16M / 64 / 50
  OCR2A = 5; // 50

  TCCR2A |= (1 << WGM21);
  // Set to CTC Mode

  TIMSK2 |= (1 << OCIE2A);
  // Set interrupt on compare match

  TCCR2B |= (1 << CS21) | (1 << CS20);
  // set prescaler to 32 and starts PWM

  sei();
}

void IsrLed::ISR_disable() {
  TCCR2A = 0;
  TCCR2B = 0;
}

float IsrLed::hue_to_rgb(float v1, float v2, float vH) {
  if (vH < 0)
    vH += 1;
  if (vH > 1)
    vH -= 1;
  if ((6 * vH) < 1)
    return (v1 + (v2 - v1) * 6 * vH);
  if ((2 * vH) < 1)
    return v2;
  if ((3 * vH) < 2)
    return (v1 + (v2 - v1) * ((2.0f / 3) - vH) * 6);
  return v1;
};

void IsrLed::set_hsl(int H, float S, float L) {
  uint8_t R, G, B;

  if (S == 0) {
    R = G = B = (LED_TYPE)(L * LED_LEVEL);
  } else {
    float v1, v2;
    float hue = (float)H / 360;

    v2 = (L < 0.5) ? (L * (1 + S)) : ((L + S) - (L * S));
    v1 = 2 * L - v2;

    R = (LED_TYPE)(LED_LEVEL * hue_to_rgb(v1, v2, hue + (1.0f / 3)));
    G = (LED_TYPE)(LED_LEVEL * hue_to_rgb(v1, v2, hue));
    B = (LED_TYPE)(LED_LEVEL * hue_to_rgb(v1, v2, hue - (1.0f / 3)));
  }
  IsrLed::_leds[0] = R;
  IsrLed::_leds[1] = G;
  IsrLed::_leds[2] = B;
  Serial.print(R);
  Serial.print(", ");
  Serial.print(G);
  Serial.print(", ");
  Serial.print(B);
  Serial.println(" ");
};

// atmega 328p port manipulation
// https://www.arduino.cc/en/Reference/PortManipulation
// Port registers B (digital pin 8 to 13)
// Port registers C (analog input pins)
// Port registers D (digital pins 0 to 7)

void IsrLed::update() {
  static volatile LED_TYPE clock_pulse = 0;
  LED_TYPE remainder = clock_pulse & LED_LEVEL;

  // set led output
  PORTC = ((IsrLed::_leds[0] > remainder) << 1) |
          ((IsrLed::_leds[2] > remainder) << 2) |
          ((IsrLed::_leds[1] > remainder) << 3);

  clock_pulse++;
}

ISR(TIMER2_COMPA_vect) { IsrLed::update(); }