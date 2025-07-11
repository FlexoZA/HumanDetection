#include "api_client.h"
#include "config.h"
#include "wifi_manager.h"
#include "power_manager.h"

// Static member initialization
HTTPClient APIClient::http;
bool APIClient::initialized = false;

void APIClient::init() {
    Serial.println("DEBUG::api_client.cpp Initializing API client...");
    
    http.setTimeout(API_TIMEOUT_MS);
    initialized = true;
    
    Serial.println("DEBUG::api_client.cpp API client initialized");
}

bool APIClient::sendDetectionEvent() {
    if (!initialized || !WiFiManager::isConnected()) {
        Serial.println("DEBUG::api_client.cpp Cannot send detection event - not initialized or no WiFi");
        return false;
    }
    
    Serial.println("DEBUG::api_client.cpp Sending detection event...");
    
    String payload = createDetectionPayload();
    return sendPostRequest(payload);
}

bool APIClient::sendHeartbeat() {
    if (!initialized || !WiFiManager::isConnected()) {
        Serial.println("DEBUG::api_client.cpp Cannot send heartbeat - not initialized or no WiFi");
        return false;
    }
    
    Serial.println("DEBUG::api_client.cpp Sending heartbeat...");
    
    String payload = createHeartbeatPayload();
    return sendPostRequest(payload);
}

bool APIClient::sendBatteryStatus(float voltage) {
    if (!initialized || !WiFiManager::isConnected()) {
        Serial.println("DEBUG::api_client.cpp Cannot send battery status - not initialized or no WiFi");
        return false;
    }
    
    Serial.println("DEBUG::api_client.cpp Sending battery status...");
    
    String payload = createBatteryPayload(voltage);
    return sendPostRequest(payload);
}

bool APIClient::testConnection() {
    if (!initialized || !WiFiManager::isConnected()) {
        Serial.println("DEBUG::api_client.cpp Cannot test connection - not initialized or no WiFi");
        return false;
    }
    
    Serial.println("DEBUG::api_client.cpp Testing connection...");
    
    http.begin(N8N_ENDPOINT_URL);
    http.addHeader("Content-Type", "application/json");
    
    int httpResponseCode = http.GET();
    
    if (httpResponseCode > 0) {
        Serial.println("DEBUG::api_client.cpp Connection test successful, response code: " + String(httpResponseCode));
        http.end();
        return true;
    } else {
        Serial.println("DEBUG::api_client.cpp Connection test failed, error: " + String(httpResponseCode));
        http.end();
        return false;
    }
}

String APIClient::createDetectionPayload() {
    DynamicJsonDocument doc(1024);
    
    doc["event_type"] = "human_detection";
    doc["device_id"] = getDeviceId();
    doc["timestamp"] = getCurrentTimestamp();
    doc["battery_voltage"] = PowerManager::getBatteryVoltage();
    doc["wifi_signal"] = WiFiManager::getSignalStrength();
    doc["location"] = "sensor_location"; // TODO: Make configurable
    
    String payload;
    serializeJson(doc, payload);
    
    return payload;
}

String APIClient::createHeartbeatPayload() {
    DynamicJsonDocument doc(1024);
    
    doc["event_type"] = "heartbeat";
    doc["device_id"] = getDeviceId();
    doc["timestamp"] = getCurrentTimestamp();
    doc["battery_voltage"] = PowerManager::getBatteryVoltage();
    doc["wifi_signal"] = WiFiManager::getSignalStrength();
    doc["uptime"] = millis();
    
    String payload;
    serializeJson(doc, payload);
    
    return payload;
}

String APIClient::createBatteryPayload(float voltage) {
    DynamicJsonDocument doc(1024);
    
    doc["event_type"] = "battery_status";
    doc["device_id"] = getDeviceId();
    doc["timestamp"] = getCurrentTimestamp();
    doc["battery_voltage"] = voltage;
    doc["battery_percentage"] = ((voltage - 3.0) / 1.2) * 100; // Rough estimate
    doc["low_battery"] = voltage < LOW_BATTERY_THRESHOLD;
    
    String payload;
    serializeJson(doc, payload);
    
    return payload;
}

bool APIClient::sendPostRequest(const String& payload) {
    http.begin(N8N_ENDPOINT_URL);
    http.addHeader("Content-Type", "application/json");
    
    Serial.println("DEBUG::api_client.cpp Sending payload: " + payload);
    
    int httpResponseCode = http.POST(payload);
    
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("DEBUG::api_client.cpp HTTP Response code: " + String(httpResponseCode));
        Serial.println("DEBUG::api_client.cpp Response: " + response);
        http.end();
        return httpResponseCode == 200;
    } else {
        Serial.println("DEBUG::api_client.cpp HTTP Request failed, error: " + String(httpResponseCode));
        http.end();
        return false;
    }
}

String APIClient::getDeviceId() {
    // Use MAC address as device ID
    uint8_t mac[6];
    WiFi.macAddress(mac);
    return String(mac[0], HEX) + ":" + String(mac[1], HEX) + ":" + 
           String(mac[2], HEX) + ":" + String(mac[3], HEX) + ":" + 
           String(mac[4], HEX) + ":" + String(mac[5], HEX);
}

String APIClient::getCurrentTimestamp() {
    // Simple timestamp based on millis() - in production, use NTP
    return String(millis());
} 