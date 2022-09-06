#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include <RingBuf.h>

#define UART_BAUDRATE 9600
#define TERMINATE_CHAR (uint8_t)255

#define DATA_LEN 8
#define DUP_DATA_LEN 3

#define MY_ID 8

struct AgentCommands_t {
  uint8_t id;
  uint8_t upper_motor, bottom_motor;
  uint8_t center_servo, outer_servo;
  uint8_t h, s, l;
};

union UartPacket_t {
  uint8_t raw[DATA_LEN];
  AgentCommands_t data;
};

class Receiver {
public:
  Receiver();

  void init();

  // True if new packet arrived
  bool available();

  AgentCommands_t read();

  void update();

protected:
  bool new_data_flag;
  bool new_packet_flag;
  RingBuf<uint8_t, DUP_DATA_LEN * DATA_LEN> _raw_buf;
  UartPacket_t _packet;

  void check_for_new_data();

  void flush();
};

#endif