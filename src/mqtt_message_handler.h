#ifndef MQTT_MESSAGE_HANDLER_H
#define MQTT_MESSAGE_HANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "command_handler.h"
#include "mqtt_client.h"

class MQTTMessageHandler {
public:
    static void init();
    static void handleMessage(const String& topic, const String& message);
    
private:
    static bool isCommandTopic(const String& topic);
    static void processCommandMessage(const String& message);
    static void sendCommandResponse(const String& command, bool success, const String& message);
};

#endif // MQTT_MESSAGE_HANDLER_H 