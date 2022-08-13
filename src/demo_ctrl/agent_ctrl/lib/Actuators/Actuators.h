#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "Arduino.h"
#include <Servo.h>

#define PIN_SERVO_X 9
#define PIN_SERVO_Y 10
#define PIN_DCMOTOR_A 5
#define PIN_DCMOTOR_B 6

#define ServoKp 0.1f
#define ServoKd 0.01f

// Gimbal angle settings
#define GEAR_RATIO_X 15.0f / 13.0f
#define GEAR_RATIO_Y 29.0f / 10.0f

#define MIN_ANGLE_X -80
#define MAX_ANGLE_X 80

#define MIN_ANGLE_Y -30
#define MAX_ANGLE_Y 30

// Servo motor phisical constaints
#define SERVO_BIAS 90.0f
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

#define constraint_float(x, a, b)                                              \
  { min(b, max(x, a)) }

// Transform from axis coordinate to servo coordinate
#define gearbox_transform(x, bias, ratio)                                      \
  { ratio *x + bias }

// gear ratio can not be 0
#define inverse_gearbox_transform(x, bias, ratio)                              \
  { (x - bias) / ratio }

#define SAMPLING_FREQ (int)(1 / 0.01f)

class Actuators {
public:
  Actuators();
  void init();

  // Set commands
  void set_angles(float eta, float xi) {
    _eta = eta;
    _xi = xi;
  };
  void set_thrusts(float w1, float w2) {
    _w1 = w1;
    _w2 = w2;
  };

  // Set the update rate of servos
  void set_update_rate(uint16_t update_rate) { _update_rate = update_rate; };

  // The main updater. This function should be called in main loop.
  void update();

protected:
  struct ServoState {
    float x;
    float dx;
  };

  struct ServoState ssX;
  struct ServoState ssY;

  static Servo servoX;
  static Servo servoY;

  uint16_t _update_rate;
  float _xi, _eta;
  float _w1, _w2;

  float _servo_model(struct ServoState &s, float command);

  float _servo_limit(float angle, const float gear_ratio, const float min_value,
                     const float max_value);
};

#endif