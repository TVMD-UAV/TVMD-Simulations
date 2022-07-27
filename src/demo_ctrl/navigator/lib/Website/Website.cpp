#include "Website.h"


Commander Website::_commander;
Controller* Website::_ctrl = NULL;
Website::Modes Website::pre_mode = None;

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
        Modes mode = (Modes)(request->getParam("mode")->value()).toInt();
        
        // Mode not change
        if (mode == pre_mode) 
            return;

        free(Website::_ctrl);
        switch (mode)
        {
        case Idle:
            // code
            Website::_ctrl = new IdleController(&Website::_commander);
            break;
        case Demo:
            // code
            Website::_ctrl = new DemoController(&Website::_commander);
            break;
        case Joystick:
            // code
            Website::_ctrl = new JoystickController(&Website::_commander);
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
            _ctrl->set_mode(Controller::Regular);
        }
        else{ // wave mode
            _ctrl->set_mode(Controller::SinWave);
        }
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("color_h")){
            Serial.println("LED顏色更換");
            int h, s, l;
            h = (request->getParam("color_h")->value()).toInt();
            s = (request->getParam("color_s")->value()).toInt();
            l = (request->getParam("color_l")->value()).toInt();
        }
        
        if (request->hasParam("upper"))
            _ctrl->set_motor(Controller::Upper, (request->getParam("upper")->value()).toInt());
        if (request->hasParam("lower"))
            _ctrl->set_motor(Controller::Lower, (request->getParam("lower")->value()).toInt());
        if (request->hasParam("center"))
            _ctrl->set_motor(Controller::Center, (request->getParam("center")->value()).toInt());
        if (request->hasParam("outer"))
            _ctrl->set_motor(Controller::Outer, (request->getParam("outer")->value()).toInt());

        if (request->hasParam("joystick_x")){
            _ctrl->set_direction((request->getParam("joystick_x")->value()).toInt(), (request->getParam("joystick_y")->value()).toInt());
        }

        request->send(200, "text/plain", "");
    });
}

void Website::update(){
    _ctrl->generate_commands();
    _commander.send_commands();
}