#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

// Forward declaration for callback function
typedef void (*MqttMessageCallback)(const String& topic, const String& message);

class MQTTClient {
public:
    static void init();
    static void loop();
    static bool connect();
    static void disconnect();
    static bool isConnected();
    
    // Publishing methods
    static bool publishDetectionEvent(const String& eventType);
    static bool publishStatus();
    static bool publishHeartbeat();
    static bool publishOnlineStatus(bool online);
    static bool publishCommandResponse(const String& command, bool success, const String& message, const String& data = "");
    
    // Subscription methods
    static void setMessageCallback(MqttMessageCallback callback);
    static void subscribeToCommands();
    
    // Connection management
    static void handleReconnect();
    static unsigned long getLastReconnectAttempt();
    static bool shouldAttemptReconnect();
    
    // Status methods
    static String getConnectionStatus();
    static String getClientId();
    static int getWiFiSignalStrength();
    static float getBatteryVoltage();
    
private:
    static WiFiClient wifiClient;
    static PubSubClient mqttClient;
    static String clientId;
    static String deviceTopic;
    static unsigned long lastReconnectAttempt;
    static unsigned long lastHeartbeat;
    static MqttMessageCallback messageCallback;
    static bool initialized;
    
    // Internal methods
    static void onMqttMessage(char* topic, byte* payload, unsigned int length);
    static String createDetectionPayload(const String& eventType);
    static String createStatusPayload();
    static String createHeartbeatPayload();
    static String getDeviceId();
    static String getCurrentTimestamp();
    static String getTopicForDevice(const String& topicSuffix);
};

#endif // MQTT_CLIENT_H 