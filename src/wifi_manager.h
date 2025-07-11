#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

class WiFiManager {
public:
    static bool init();
    static bool connect();
    static void disconnect();
    static bool isConnected();
    static int getSignalStrength();
    static String getLocalIP();
    
private:
    static bool connectToWiFi();
    static void printConnectionInfo();
};

#endif // WIFI_MANAGER_H 