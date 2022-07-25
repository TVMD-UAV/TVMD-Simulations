#include "Website.h"

Website::Website():
server(80)
{

}

void Website::init(){
    spiffs_init();
    routing();
    server.begin();
}

void Website::spiffs_init(){
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    else{
        Serial.println("SPIFFS mounted");
    }
}

void Website::routing(){
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html");
    });

    server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/jquery.min.js");
    });

    server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request){
        int mode = (request->getParam("mode")->value()).toInt();
        switch (mode)
        {
        case Idle:
            // code
            break;
        case Demo:
            // code
            break;
        case Joystick:
            // code
            break;
        case Fixwing:
            // code
            break;
        
        default:
            break;
        }
    });

    server.on("/submode", HTTP_GET, [](AsyncWebServerRequest *request){
        int submode = (request->getParam("submode")->value()).toInt();
        if(submode==1){ // fixed mode

        }
        else{ // wave mode

        }
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("b_color_r")){
            Serial.println("全體LED顏色更換");
            uint8_t r, g, b;
            r = uint8_t((request->getParam("b_color_r")->value()).toInt());
            g = uint8_t((request->getParam("b_color_g")->value()).toInt());
            b = uint8_t((request->getParam("b_color_b")->value()).toInt());

            /*for(int i=0; i<N_PACKETS; i++){
                packets[i].data.r = r;
                packets[i].data.g = g;
                packets[i].data.b = b;
            }*/
        }

        if (request->hasParam("id")){
            Serial.println("單一模組調整");
            uint8_t id = uint8_t((request->getParam("id")->value()).toInt());
            /*packets[id].data.id = id;
            packets[id].data.upper_motor = uint8_t((request->getParam("upper")->value()).toInt());
            packets[id].data.bottom_motor = uint8_t((request->getParam("bottom")->value()).toInt());
            packets[id].data.center_servo = uint8_t((request->getParam("center")->value()).toInt());
            packets[id].data.outer_servo = uint8_t((request->getParam("outer")->value()).toInt());
            packets[id].data.r = uint8_t((request->getParam("color_r")->value()).toInt());
            packets[id].data.g = uint8_t((request->getParam("color_g")->value()).toInt());
            packets[id].data.b = uint8_t((request->getParam("color_b")->value()).toInt());*/
        }


        request->send(200, "text/plain", "");
    });
}