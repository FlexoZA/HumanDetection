#include "mqtt_message_handler.h"

// External function to trigger command received visual indicator
extern void triggerCommandReceivedIndicator();

void MQTTMessageHandler::init() {
    Serial.println("DEBUG::mqtt_message_handler.cpp MQTT message handler initialized");
}

void MQTTMessageHandler::handleMessage(const String& topic, const String& message) {
    Serial.println("DEBUG::mqtt_message_handler.cpp Handling message from topic: " + topic);
    
    if (isCommandTopic(topic)) {
        processCommandMessage(message);
    } else {
        Serial.println("DEBUG::mqtt_message_handler.cpp Unknown topic, ignoring message");
    }
}

bool MQTTMessageHandler::isCommandTopic(const String& topic) {
    // Check if topic ends with "/command"
    return topic.endsWith(MQTT_TOPIC_COMMAND);
}

void MQTTMessageHandler::processCommandMessage(const String& message) {
    Serial.println("DEBUG::mqtt_message_handler.cpp Processing command message: " + message);
    Serial.println("DEBUG::mqtt_message_handler.cpp Message length: " + String(message.length()));
    
    // Show visual indicator for command received
    triggerCommandReceivedIndicator();
    
    // Parse the JSON message
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        Serial.println("DEBUG::mqtt_message_handler.cpp Failed to parse JSON command");
        Serial.println("DEBUG::mqtt_message_handler.cpp JSON Error: " + String(error.c_str()));
        sendCommandResponse("unknown", false, "Invalid JSON format");
        return;
    }
    
    Serial.println("DEBUG::mqtt_message_handler.cpp JSON parsed successfully");
    
    // Extract command details
    String action = doc["action"] | "";
    String source = doc["source"] | "mqtt";
    String apiKey = doc["api_key"] | "";
    
    Serial.println("DEBUG::mqtt_message_handler.cpp Extracted action: '" + action + "'");
    Serial.println("DEBUG::mqtt_message_handler.cpp Extracted source: '" + source + "'");
    Serial.println("DEBUG::mqtt_message_handler.cpp API key present: " + String(apiKey.length() > 0 ? "Yes" : "No"));
    
    if (action.isEmpty()) {
        Serial.println("DEBUG::mqtt_message_handler.cpp No action specified in command");
        sendCommandResponse("unknown", false, "No action specified");
        return;
    }
    
    Serial.println("DEBUG::mqtt_message_handler.cpp Calling CommandHandler::processCommand()");
    
    // Process the command using existing CommandHandler
    CommandResponse response = CommandHandler::processCommand(message);
    
    Serial.println("DEBUG::mqtt_message_handler.cpp CommandHandler response - Success: " + String(response.success ? "true" : "false"));
    Serial.println("DEBUG::mqtt_message_handler.cpp CommandHandler response - Message: " + response.message);
    
    // Send response back via MQTT
    sendCommandResponse(action, response.success, response.message);
    
    // Log the result
    if (response.success) {
        Serial.println("DEBUG::mqtt_message_handler.cpp Command executed successfully: " + action);
    } else {
        Serial.println("DEBUG::mqtt_message_handler.cpp Command failed: " + action + " - " + response.message);
    }
}

void MQTTMessageHandler::sendCommandResponse(const String& command, bool success, const String& message) {
    // Use the MQTT client to send the response
    MQTTClient::publishCommandResponse(command, success, message);
} 