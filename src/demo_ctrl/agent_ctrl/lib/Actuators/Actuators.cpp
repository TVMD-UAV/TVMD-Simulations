#include <Actuators.h>

Servo Actuators::servoX;
Servo Actuators::servoY;

Actuators::Actuators() { _update_rate = 1000; }

void Actuators::init() {
  pinMode(PIN_DCMOTOR_A, OUTPUT);
  pinMode(PIN_DCMOTOR_B, OUTPUT);
  digitalWrite(PIN_DCMOTOR_A, LOW);
  digitalWrite(PIN_DCMOTOR_B, LOW);

  // Attach servos
  Actuators::servoX.attach(PIN_SERVO_X);
  Actuators::servoY.attach(PIN_SERVO_Y);

  set_angles(0, 0);
  set_thrusts(0, 0);
  set_update_rate(SAMPLING_FREQ);
}

void Actuators::update() {
  static auto last_call = millis();
  if (millis() - last_call > 1000 / _update_rate) {
    last_call = millis();

    // Simulating the servo motors as linear systems
    float x_angle = _servo_model(ssX, _eta);
    float y_angle = _servo_model(ssY, _xi);

    // Constraint the maximum and minimum rotating angle based on xi and eta
    x_angle = _servo_limit(x_angle, GEAR_RATIO_X, MIN_ANGLE_X, MAX_ANGLE_X);
    y_angle = _servo_limit(y_angle, GEAR_RATIO_Y, MIN_ANGLE_Y, MAX_ANGLE_Y);

    // Gearbox and bias (coordinate transform)
    float servo_x_angle = gearbox_transform(x_angle, 90.0f, GEAR_RATIO_X);
    float servo_y_angle = gearbox_transform(y_angle, 90.0f, GEAR_RATIO_Y);

    // Send commands to servos
    Actuators::servoX.write(servo_x_angle);
    Actuators::servoY.write(servo_y_angle);

    // Send commands to DC motors
    analogWrite(PIN_DCMOTOR_A, _w1);
    analogWrite(PIN_DCMOTOR_B, _w2);
  }
}

float Actuators::_servo_model(struct ServoState &s, float command) {
  // The following discrete state space model is obtained with sampling time =
  // 0.01
  float n_state_x = 0.9995 * s.x + 0.00995 * s.dx + 0.0005 * command;
  float n_state_v = -0.0995 * s.x + 0.9896 * s.dx + 0.0995 * command;
  s.x = n_state_x;
  s.dx = n_state_v;
  return s.x;
}

float Actuators::_servo_limit(float angle, const float gear_ratio,
                              const float min_value, const float max_value) {
  const float x_phisical_min =
      inverse_gearbox_transform(SERVO_MIN_ANGLE, SERVO_BIAS, gear_ratio);
  const float x_phisical_max =
      inverse_gearbox_transform(SERVO_MAX_ANGLE, SERVO_BIAS, gear_ratio);
  const float x_angle_min = max(x_phisical_min, min_value);
  const float x_angle_max = min(x_phisical_max, max_value);
  return constraint_float(angle, x_angle_min, x_angle_max);
}