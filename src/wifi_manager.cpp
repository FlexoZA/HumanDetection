#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>

bool WiFiManager::init() {
    Serial.println("DEBUG::wifi_manager.cpp Initializing WiFi...");
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    return connectToWiFi();
}

bool WiFiManager::connect() {
    if (isConnected()) {
        return true;
    }
    
    Serial.println("DEBUG::wifi_manager.cpp Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    return connectToWiFi();
}

void WiFiManager::disconnect() {
    Serial.println("DEBUG::wifi_manager.cpp Disconnecting from WiFi...");
    WiFi.disconnect();
}

bool WiFiManager::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

int WiFiManager::getSignalStrength() {
    if (!isConnected()) {
        return -999;
    }
    return WiFi.RSSI();
}

String WiFiManager::getLocalIP() {
    if (!isConnected()) {
        return "0.0.0.0";
    }
    return WiFi.localIP().toString();
}

bool WiFiManager::connectToWiFi() {
    unsigned long startTime = millis();
    
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT_MS) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        printConnectionInfo();
        return true;
    } else {
        Serial.println();
        Serial.println("DEBUG::wifi_manager.cpp WiFi connection failed!");
        return false;
    }
}

void WiFiManager::printConnectionInfo() {
    Serial.println("DEBUG::wifi_manager.cpp WiFi connected successfully!");
    Serial.println("DEBUG::wifi_manager.cpp IP address: " + WiFi.localIP().toString());
    Serial.println("DEBUG::wifi_manager.cpp Signal strength: " + String(WiFi.RSSI()) + " dBm");
} 