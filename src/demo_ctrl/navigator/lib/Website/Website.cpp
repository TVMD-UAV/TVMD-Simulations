#include "Website.h"


Commander Website::_commander;
Controller *Website::_ctrl = NULL;
Controller *Website::_ctrl_ready = NULL;
Website::Modes Website::pre_mode = None;
Website::ControllerState Website::controller_state = ControllerState::Normal;

Website::Website() : server(80) {}

void Website::init()
{
    spiffs_init();
    routing();
    server.begin();
}

void Website::spiffs_init()
{
    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    } else {
        Serial.println("SPIFFS mounted");
    }
}

void Website::routing()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html");
    });

    server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/jquery.min.js");
    });

    server.on("/joy.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/joy.js");
    });

    server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request) {
        Modes mode = (Modes)(request->getParam("mode")->value()).toInt();

        // Mode not change
        if (mode == pre_mode) {
            request->send(200, "text/plain", "");
            return;
        }

        if (Website::_ctrl_ready)
            delete Website::_ctrl_ready;

        switch (mode) {
        case Idle:
            Website::_ctrl_ready = new IdleController(&Website::_commander);
            break;
        case Demo:
            Website::_ctrl_ready = new DemoController(&Website::_commander);
            break;
        case Joystick:
            Website::_ctrl_ready = new JoystickController(&Website::_commander);
            break;

        default:
            break;
        }
        controller_state = ControllerState::Ready;
        Serial.println("get mode req");
        request->send(200, "text/plain", "");
        return;
    });

    server.on("/submode", HTTP_GET, [](AsyncWebServerRequest *request) {
        int submode = (request->getParam("submode")->value()).toInt();
        int mode;
        switch (_ctrl->controller_type) {
        case Demo:
            mode = (submode == 1) ? MotorMode::Fixed : MotorMode::SineWave;
            _ctrl->parse_input(&mode, 1);
            break;

        case Idle:
            _ctrl->set_led_mode((submode == 1) ? Controller::Regular
                                               : Controller::SinWave);

        case JoyStick:
            mode = (submode == 1) ? Hover : FixedWing;
            _ctrl->parse_input(&mode, 1);
            break;
        }
        Serial.println("get submode req");
        request->send(200, "text/plain", "");
        return;
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (_ctrl == NULL) {
            request->send(200, "text/plain", "");
            return;
        }
        if (request->hasParam("color_h")) {
            Serial.println("LED顏色更換");
            int h, s, l;
            h = (request->getParam("color_h")->value()).toInt();
            s = (request->getParam("color_s")->value()).toInt();
            l = (request->getParam("color_l")->value()).toInt();
            _ctrl->set_hsl(h, s, l);
        }

        if (request->hasParam("freq"))
            _ctrl->set_led_freq((request->getParam("freq")->value()).toFloat());

        // Demo mode
        if (_ctrl->controller_type == ControllerType::Demo) {
            int input[2];
            if (request->hasParam("upper")) {
                input[0] = MotorType::Upper;
                input[1] = (request->getParam("upper")->value()).toInt();
            } else if (request->hasParam("lower")) {
                input[0] = MotorType::Lower;
                input[1] = (request->getParam("lower")->value()).toInt();
            } else if (request->hasParam("center")) {
                input[0] = MotorType::Center;
                input[1] = (request->getParam("center")->value()).toInt();
            } else if (request->hasParam("outer")) {
                input[0] = MotorType::Outer;
                input[1] = (request->getParam("outer")->value()).toInt();
            }
            _ctrl->parse_input(input, 2);
        }

        // JoyStick mode
        if (request->hasParam("ux")) {
            int inputs[6];
            inputs[0] = (request->getParam("ux")->value()).toInt();
            inputs[1] = (request->getParam("uy")->value()).toInt();
            inputs[2] = (request->getParam("uz")->value()).toInt();
            inputs[3] = (request->getParam("mx")->value()).toInt();
            inputs[4] = (request->getParam("my")->value()).toInt();
            inputs[5] = (request->getParam("mz")->value()).toInt();
            _ctrl->parse_input(inputs, 6);
        }

        request->send(200, "text/plain", "");
        return;
    });
}

void Website::update()
{
    if (controller_state == ControllerState::Ready) {
        // Delete original controller
        if (_ctrl != NULL)
            delete _ctrl;
        _ctrl = _ctrl_ready;
        _ctrl_ready = NULL;
        controller_state = ControllerState::Normal;
    } else if (_ctrl != NULL) {
        _ctrl->update();
    }
}