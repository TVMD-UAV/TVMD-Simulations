#include "Commander.h"

Commander::Commander():
    buffer_state(false){
}

void Commander::init(){
    buf = buf1;
    buffer_state = false;
    Serial2.begin(UART_BAUDRATE);
}

void Commander::set_agent_commands(const AgentCommands_t * const com){
    UartPacket_t *mirror = (buffer_state) ? buf1 : buf2;

    if (com->id == BROADCAST_ID){
        for (uint8_t i=0; i<NUM_AGENT; i++){
            memcpy(mirror[i].raw, (void*)com, sizeof(AgentCommands_t));
            mirror[i].data.id = i+1;
        }
    }
    else{
        memcpy(mirror[com->id].raw, (void*)com, sizeof(AgentCommands_t));
    }
}

void Commander::update_packets(){
    if(buffer_state){
        //buf2
        buf = buf2;
    }
    else{
        //buf1
        buf = buf1;
    }
    buffer_state = !buffer_state;
}

void Commander::send_commands(){
    for (uint8_t i=0; i<NUM_AGENT; i++){
        // Headers
        Serial2.write(HEADER_CHAR);
        Serial2.write(HEADER_CHAR);
        for (uint8_t j=0; j<DATA_LEN; j++){
            Serial2.write(buf[i].raw[j]);
        }
    }
}