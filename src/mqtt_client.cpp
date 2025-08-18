#include "mqtt_client.h"
#include <time.h>

// Static member definitions
WiFiClient MQTTClient::wifiClient;
PubSubClient MQTTClient::mqttClient(wifiClient);
String MQTTClient::clientId = "";
String MQTTClient::deviceTopic = "";
unsigned long MQTTClient::lastReconnectAttempt = 0;
unsigned long MQTTClient::lastHeartbeat = 0;
MqttMessageCallback MQTTClient::messageCallback = nullptr;
bool MQTTClient::initialized = false;

void MQTTClient::init() {
    if (initialized) return;
    
    // Generate unique client ID using MAC address
    clientId = String(MQTT_CLIENT_ID_PREFIX) + getDeviceId();
    deviceTopic = String(MQTT_TOPIC_BASE) + "device_" + getDeviceId();
    
    // Configure MQTT client
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(onMqttMessage);
    mqttClient.setKeepAlive(MQTT_KEEP_ALIVE);
    
    // Set buffer sizes if needed
    mqttClient.setBufferSize(1024);
    
    Serial.println("DEBUG::mqtt_client.cpp MQTT client initialized");
    Serial.println("DEBUG::mqtt_client.cpp Client ID: " + clientId);
    Serial.println("DEBUG::mqtt_client.cpp Device Topic: " + deviceTopic);
    
    initialized = true;
}

void MQTTClient::loop() {
    if (!initialized) return;
    
    if (isConnected()) {
        mqttClient.loop();
        
        // Send periodic heartbeat
        if (millis() - lastHeartbeat > MQTT_HEARTBEAT_INTERVAL) {
            publishHeartbeat();
            lastHeartbeat = millis();
        }
    } else {
        // Handle reconnection
        if (shouldAttemptReconnect()) {
            handleReconnect();
        }
    }
}

bool MQTTClient::connect() {
    if (!initialized || !WiFi.isConnected()) {
        Serial.println("DEBUG::mqtt_client.cpp Cannot connect MQTT - WiFi not ready or MQTT not initialized");
        return false;
    }
    
    Serial.println("DEBUG::mqtt_client.cpp Attempting MQTT connection...");
    Serial.println("DEBUG::mqtt_client.cpp Broker: " + String(MQTT_BROKER) + ":" + String(MQTT_PORT));
    Serial.println("DEBUG::mqtt_client.cpp Client ID: " + clientId);
    Serial.println("DEBUG::mqtt_client.cpp Username: " + String(MQTT_USERNAME));
    
    // Configure Last Will and Testament so broker publishes 'offline' on unexpected disconnects
    String lwt_topic = getTopicForDevice(MQTT_TOPIC_ONLINE);
    // Set LWT payload with device_id so consumers can parse the ID from the message body
    String lwt_payload;
    {
        DynamicJsonDocument willDoc(256);
        willDoc["device_id"] = getDeviceId();
        willDoc["message"] = "offline";
        serializeJson(willDoc, lwt_payload);
    }

    bool connected = mqttClient.connect(
        clientId.c_str(),
        MQTT_USERNAME,
        MQTT_PASSWORD,
        lwt_topic.c_str(),
        0,          // will QoS
        true,       // will retained
        lwt_payload.c_str()   // will payload as JSON
    );
    
    if (connected) {
        Serial.println("DEBUG::mqtt_client.cpp MQTT connection (with LWT) successful");
    }
    
    if (connected) {
        Serial.println("DEBUG::mqtt_client.cpp MQTT connected successfully");
        
        // Publish online status
        publishOnlineStatus(true);
        
        // Subscribe to command topic
        subscribeToCommands();
        
        // Send initial status
        publishStatus();
        
        lastHeartbeat = millis();
    } else {
        Serial.println("DEBUG::mqtt_client.cpp MQTT connection failed, rc=" + String(mqttClient.state()));
    }
    
    return connected;
}

void MQTTClient::disconnect() {
    if (isConnected()) {
        publishOnlineStatus(false);
        mqttClient.disconnect();
        Serial.println("DEBUG::mqtt_client.cpp MQTT disconnected");
    }
}

bool MQTTClient::isConnected() {
    return initialized && mqttClient.connected();
}

bool MQTTClient::publishDetectionEvent(const String& eventType) {
    if (!isConnected()) return false;
    
    String topic = getTopicForDevice(MQTT_TOPIC_DETECTION);
    String payload = createDetectionPayload(eventType);
    
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), MQTT_QOS_DETECTION);
    
    if (success) {
        Serial.println("DEBUG::mqtt_client.cpp Detection event published: " + eventType);
    } else {
        Serial.println("DEBUG::mqtt_client.cpp Failed to publish detection event: " + eventType);
    }
    
    return success;
}

bool MQTTClient::publishStatus() {
    if (!isConnected()) return false;
    
    String topic = getTopicForDevice(MQTT_TOPIC_STATUS);
    String payload = createStatusPayload();
    
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), MQTT_QOS_STATUS);
    
    if (success) {
        Serial.println("DEBUG::mqtt_client.cpp Status published");
    } else {
        Serial.println("DEBUG::mqtt_client.cpp Failed to publish status");
    }
    
    return success;
}

bool MQTTClient::publishHeartbeat() {
    if (!isConnected()) return false;
    
    String topic = getTopicForDevice(MQTT_TOPIC_HEARTBEAT);
    String payload = createHeartbeatPayload();
    
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), MQTT_QOS_STATUS);
    
    if (success) {
        Serial.println("DEBUG::mqtt_client.cpp Heartbeat published");
    }
    
    return success;
}

bool MQTTClient::publishOnlineStatus(bool online) {
    if (!mqttClient.connected() && online) return false;
    
    String topic = getTopicForDevice(MQTT_TOPIC_ONLINE);
    
    // Publish a JSON payload with both message and device_id, retained
    DynamicJsonDocument doc(256);
    doc["device_id"] = getDeviceId();
    doc["message"] = online ? "online" : "offline";
    String payload;
    serializeJson(doc, payload);
    
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), true); // retained
    
    if (success) {
        Serial.println("DEBUG::mqtt_client.cpp Online status published: " + String(online ? "online" : "offline"));
    }
    
    return success;
}

bool MQTTClient::publishCommandResponse(const String& command, bool success, const String& message, const String& data) {
    if (!isConnected()) return false;
    
    String topic = getTopicForDevice(MQTT_TOPIC_STATUS);
    
    // Create response payload
    DynamicJsonDocument doc(1024);
    doc["device_id"] = getDeviceId();
    doc["command_response"] = command;
    doc["success"] = success;
    doc["message"] = message;
    // Timestamp: ISO8601 when available; empty if not yet synced
    {
        time_t now = time(nullptr);
        if (now > 1600000000) {
            doc["timestamp_iso"] = getCurrentTimestamp();
        } else {
            doc["timestamp_iso"] = "";
        }
    }
    
    // Merge additional data JSON into top-level if provided
    if (data.length() > 0) {
        DynamicJsonDocument dataDoc(1024);
        DeserializationError err = deserializeJson(dataDoc, data);
        if (!err) {
            JsonObject dataObj = dataDoc.as<JsonObject>();
            for (JsonPair kv : dataObj) {
                doc[kv.key()] = kv.value();
            }
        } else {
            // If not valid JSON, include raw data string for debugging
            doc["data"] = data;
        }
    }
    
    String payload;
    serializeJson(doc, payload);
    
    bool published = mqttClient.publish(topic.c_str(), payload.c_str(), MQTT_QOS_STATUS);
    
    if (published) {
        Serial.println("DEBUG::mqtt_client.cpp Command response published: " + command);
    }
    
    return published;
}

void MQTTClient::setMessageCallback(MqttMessageCallback callback) {
    messageCallback = callback;
}

void MQTTClient::subscribeToCommands() {
    if (!isConnected()) return;
    
    String commandTopic = getTopicForDevice(MQTT_TOPIC_COMMAND);
    
    bool success = mqttClient.subscribe(commandTopic.c_str(), MQTT_QOS_COMMANDS);
    
    if (success) {
        Serial.println("DEBUG::mqtt_client.cpp Subscribed to commands: " + commandTopic);
    } else {
        Serial.println("DEBUG::mqtt_client.cpp Failed to subscribe to commands");
    }
}

void MQTTClient::handleReconnect() {
    lastReconnectAttempt = millis();
    
    Serial.println("DEBUG::mqtt_client.cpp Attempting MQTT reconnection...");
    
    if (connect()) {
        Serial.println("DEBUG::mqtt_client.cpp MQTT reconnected successfully");
    } else {
        Serial.println("DEBUG::mqtt_client.cpp MQTT reconnection failed, will retry in " + String(MQTT_RECONNECT_INTERVAL/1000) + " seconds");
    }
}

unsigned long MQTTClient::getLastReconnectAttempt() {
    return lastReconnectAttempt;
}

bool MQTTClient::shouldAttemptReconnect() {
    return (millis() - lastReconnectAttempt > MQTT_RECONNECT_INTERVAL);
}

String MQTTClient::getConnectionStatus() {
    if (!initialized) return "Not initialized";
    if (!WiFi.isConnected()) return "WiFi disconnected";
    if (!isConnected()) return "MQTT disconnected";
    return "Connected";
}

String MQTTClient::getClientId() {
    return clientId;
}

int MQTTClient::getWiFiSignalStrength() {
    return WiFi.RSSI();
}

float MQTTClient::getBatteryVoltage() {
    // Placeholder - implement based on your battery monitoring setup
    return 3.7; // Default voltage
}

// Private methods
void MQTTClient::onMqttMessage(char* topic, byte* payload, unsigned int length) {
    // Convert payload to string
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    String topicStr = String(topic);
    
    Serial.println("DEBUG::mqtt_client.cpp ========== MQTT MESSAGE RECEIVED ==========");
    Serial.println("DEBUG::mqtt_client.cpp Topic: " + topicStr);
    Serial.println("DEBUG::mqtt_client.cpp Message length: " + String(length));
    Serial.println("DEBUG::mqtt_client.cpp Message: " + message);
    Serial.println("DEBUG::mqtt_client.cpp Raw payload bytes:");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print("DEBUG::mqtt_client.cpp   [" + String(i) + "]: " + String((int)payload[i]) + " ('" + String((char)payload[i]) + "')");
        Serial.println();
    }
    Serial.println("DEBUG::mqtt_client.cpp ============================================");
    
    // Call registered callback if available
    if (messageCallback != nullptr) {
        Serial.println("DEBUG::mqtt_client.cpp Calling message callback function");
        messageCallback(topicStr, message);
    } else {
        Serial.println("DEBUG::mqtt_client.cpp No message callback registered!");
    }
}

String MQTTClient::createDetectionPayload(const String& eventType) {
    DynamicJsonDocument doc(512);
    
    doc["event_type"] = eventType;
    doc["device_id"] = getDeviceId();
    {
        time_t now = time(nullptr);
        if (now > 1600000000) {
            doc["timestamp_iso"] = getCurrentTimestamp();
        } else {
            doc["timestamp_iso"] = "";
        }
    }
    doc["battery_voltage"] = getBatteryVoltage();
    doc["wifi_signal"] = getWiFiSignalStrength();
    doc["location"] = "sensor_location";
    
    String payload;
    serializeJson(doc, payload);
    
    return payload;
}

String MQTTClient::createStatusPayload() {
    DynamicJsonDocument doc(768);
    
    doc["device_id"] = getDeviceId();
    doc["wifi_connected"] = WiFi.isConnected();
    doc["mqtt_connected"] = isConnected();
    doc["uptime_ms"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["wifi_signal"] = getWiFiSignalStrength();
    doc["battery_voltage"] = getBatteryVoltage();
    {
        time_t now = time(nullptr);
        if (now > 1600000000) {
            doc["timestamp_iso"] = getCurrentTimestamp();
        } else {
            doc["timestamp_iso"] = "";
        }
    }
    
    String payload;
    serializeJson(doc, payload);
    
    return payload;
}

String MQTTClient::createHeartbeatPayload() {
    DynamicJsonDocument doc(256);
    
    doc["device_id"] = getDeviceId();
    {
        time_t now = time(nullptr);
        if (now > 1600000000) {
            doc["timestamp_iso"] = getCurrentTimestamp();
        } else {
            doc["timestamp_iso"] = "";
        }
    }
    doc["uptime_ms"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["wifi_signal"] = getWiFiSignalStrength();
    
    String payload;
    serializeJson(doc, payload);
    
    return payload;
}

String MQTTClient::getDeviceId() {
    String mac = WiFi.macAddress();
    mac.replace(":", "");  // Remove colons for cleaner device ID
    mac.toLowerCase();     // Convert to lowercase for consistency
    return mac;
}

String MQTTClient::getCurrentTimestamp() {
    // If SNTP time is available, return ISO 8601 local time with offset; otherwise fall back to seconds since boot
    time_t now = time(nullptr);
    if (now > 1600000000) {
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        char buf[32];
        // Format: YYYY-MM-DDTHH:MM:SS+02:00 (SAST)
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
        String ts(buf);
        ts += TIME_TZ_OFFSET_STR; // from config.h
        return ts;
    }
    return String(millis() / 1000);
}

String MQTTClient::getTopicForDevice(const String& topicSuffix) {
    return deviceTopic + topicSuffix;
} 