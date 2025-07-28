#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"

enum CommandType {
    CMD_ARM,
    CMD_DISARM, 
    CMD_STATUS,
    CMD_TEST_LEDS,
    CMD_REBOOT,
    CMD_UNKNOWN
};

struct CommandResponse {
    bool success;
    String message;
    String data;
    int httpCode;
};

class CommandHandler {
public:
    static void init();
    static CommandResponse processCommand(const String& jsonCommand);
    static CommandType parseCommandType(const String& action);
    static String createStatusResponse();
    static CommandResponse handleStatusCommand();
    
private:
    static CommandResponse handleArmCommand(const String& source);
    static CommandResponse handleDisarmCommand(const String& source);
    static CommandResponse handleTestLedsCommand();
    static CommandResponse handleRebootCommand();
    static bool validateApiKey(const String& providedKey);
};

#endif // COMMAND_HANDLER_H 