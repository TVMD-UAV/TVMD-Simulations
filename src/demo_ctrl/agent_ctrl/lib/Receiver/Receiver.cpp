#include "Receiver.h"

Receiver::Receiver():
new_data_flag(false), new_packet_flag(false){

}

void Receiver::init(){
    Serial.begin(UART_BAUDRATE);
}

void Receiver::check_for_new_data(){
    if (Serial.available()){
        uint8_t c = Serial.read();

        // Clear old data if the buffer is full
        if (_raw_buf.isFull()){
            flush();
        }
        _raw_buf.push(c);
        if (c == TERMINATE_CHAR){
            new_data_flag = true;
        }
    }
}

bool Receiver::available(){
    if (new_data_flag){
        new_data_flag = false;
        if (_raw_buf.size() >= DATA_LEN){
            // read until hit TERMINATOR
            uint8_t c, i;
            _raw_buf.pop(c);
            for (i=0; c != TERMINATE_CHAR && i < DATA_LEN; i++){
                _packet.raw[i] = c;
                _raw_buf.pop(c);
            }

            if (c==TERMINATE_CHAR){
                if (_packet.data.id == 0 || _packet.data.id == MY_ID){
                    new_packet_flag = true;
                    return true;
                }
            }

            flush();
        }
    }
    return false;
}

AgentCommands_t Receiver::read(){
    new_packet_flag = false;
    return _packet.data;
}

void Receiver::update(){
    check_for_new_data();
}

void Receiver::flush(){
    // Pop until reaching TERMINATOR
    uint8_t c = 0;
    while (!_raw_buf.isEmpty() && c != TERMINATE_CHAR){
        _raw_buf.pop(c);
    }
}