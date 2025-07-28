#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "command_handler.h"

class HTTPServer {
public:
    static void init();
    static void handleClient();
    static bool isRunning();
    static String getServerIP();
    
private:
    static WebServer server;
    static bool serverRunning;
    
    static void handleCommand();
    static void handleStatus();
    static void handleRoot();
    static void handleNotFound();
    static void sendJsonResponse(int code, bool success, const String& message, const String& data = "");
    static void setCORSHeaders();
};

#endif // HTTP_SERVER_H 