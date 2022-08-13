#include <Actuators.h>
#include <Arduino.h>
#include <IsrLed.h>
#include <Receiver.h>

IsrLed leds = IsrLed();
Actuators actuator = Actuators();
Receiver receiver = Receiver();

void demo_program();

void setup() {
  // put your setup code here, to run once:
  leds.init();
  actuator.init();
  receiver.init();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*static auto last_update = millis();
  if (millis() - last_update > 10){
      last_update = millis();
      demo_program();
      if (Serial.available()){
          if (Serial.read() == 'A'){
              leds.set_hsl(1 % 360, 1, 0.5);
          }
          else{
              leds.set_hsl(1 % 360, 0, 0);
          }
      }
  }*/

  if (receiver.available()) {
    AgentCommands_t com = receiver.read();

    // set leds
    int h = 360 * (float)com.h / 254.0f;
    float s = (float)com.s / 254.0f;
    float l = (float)com.l / 254.0f;
    leds.set_hsl(h, s, l);
    Serial.print(h);
    Serial.print(", ");
    Serial.print(s);
    Serial.print(", ");
    Serial.print(l);
    Serial.println(" ");

    // set servos
    float sx = ((int)com.outer_servo - 127) * 90.0f / 127.0f;
    float sy = ((int)com.center_servo - 127) * 90.0f / 127.0f;
    actuator.set_angles(sx, sy);

    // set actuators
    actuator.set_thrusts(com.upper_motor, com.bottom_motor);
  }
  actuator.update();
  receiver.update();
}

void demo_program() {
  int color = 360 * (double)millis() / 1000 / 3.1415 / 2;
  leds.set_hsl(color % 360, 1, 0.5);

  float mx = 200 * (sin((float)millis() / 1000) + 1);
  float my = 200 * (cos((float)millis() / 1000) + 1);

  float sx = 10 * (sin((float)millis() / 1000));
  float sy = 10 * (cos((float)millis() / 1000));

  actuator.set_angles(sx, sy);
  actuator.set_thrusts(mx, my);
  // actuator.set_angles(0, 0);
  // actuator.set_thrusts(0, 0);
}