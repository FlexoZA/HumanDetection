#ifndef API_CLIENT_H
#define API_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

class APIClient {
public:
    static void init();
    static bool sendDetectionEvent();
    static bool sendAutoArmedEvent();
    static bool sendHeartbeat();
    static bool sendBatteryStatus(float voltage);
    static bool testConnection();
    
private:
    static HTTPClient http;
    static bool initialized;
    
    static String createDetectionPayload();
    static String createAutoArmedPayload();
    static String createHeartbeatPayload();
    static String createBatteryPayload(float voltage);
    static bool sendPostRequest(const String& payload);
    static String getDeviceId();
    static String getCurrentTimestamp();
};

#endif // API_CLIENT_H 