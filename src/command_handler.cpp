#include "command_handler.h"
#include <FastLED.h>
#include <WiFi.h>
#include "wifi_manager.h"

// Forward declarations from main.cpp
enum SystemMode {
    DISARMED,   // System is disarmed - no alerts, just visual feedback
    ARMED       // System is armed - detection triggers notifications
};

// External references to main.cpp variables
extern SystemMode currentMode;
extern CRGB leds[];
extern void clearAllLEDs();
extern void showModeIndicator();
extern unsigned long lastMovementTime;

void CommandHandler::init() {
    Serial.println("DEBUG::command_handler.cpp Command handler initialized");
}

CommandResponse CommandHandler::processCommand(const String& jsonCommand) {
    Serial.println("DEBUG::command_handler.cpp ========== PROCESSING COMMAND ==========");
    Serial.println("DEBUG::command_handler.cpp Received JSON: " + jsonCommand);
    
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, jsonCommand);
    
    if (error) {
        Serial.println("DEBUG::command_handler.cpp Invalid JSON received");
        Serial.println("DEBUG::command_handler.cpp JSON Error: " + String(error.c_str()));
        return {false, "Invalid JSON format", "", 400};
    }
    
    Serial.println("DEBUG::command_handler.cpp JSON parsed successfully");
    
    // Validate API key
    String apiKey = doc["api_key"] | "";
    Serial.println("DEBUG::command_handler.cpp API Key received: '" + apiKey + "'");
    Serial.println("DEBUG::command_handler.cpp API Key length: " + String(apiKey.length()));
    
    if (!validateApiKey(apiKey)) {
        Serial.println("DEBUG::command_handler.cpp Invalid API key - validation failed");
        return {false, "Invalid API key", "", 401};
    }
    
    Serial.println("DEBUG::command_handler.cpp API key validated successfully");
    
    // Parse command
    String action = doc["action"] | "";
    String source = doc["source"] | "unknown";
    
    Serial.println("DEBUG::command_handler.cpp Extracted action: '" + action + "'");
    Serial.println("DEBUG::command_handler.cpp Extracted source: '" + source + "'");
    
    CommandType cmdType = parseCommandType(action);
    Serial.println("DEBUG::command_handler.cpp Parsed command type: " + String((int)cmdType));
    
    Serial.println("DEBUG::command_handler.cpp Processing command: " + action + " from " + source);
    
    switch (cmdType) {
        case CMD_ARM:
            Serial.println("DEBUG::command_handler.cpp Executing ARM command");
            return handleArmCommand(source);
        case CMD_DISARM:
            Serial.println("DEBUG::command_handler.cpp Executing DISARM command");
            return handleDisarmCommand(source);
        case CMD_STATUS:
            Serial.println("DEBUG::command_handler.cpp Executing STATUS command");
            return handleStatusCommand();
        case CMD_TEST_LEDS:
            Serial.println("DEBUG::command_handler.cpp Executing TEST_LEDS command");
            return handleTestLedsCommand();
        case CMD_REBOOT:
            Serial.println("DEBUG::command_handler.cpp Executing REBOOT command");
            return handleRebootCommand();
        default:
            Serial.println("DEBUG::command_handler.cpp Unknown command type - returning error");
            return {false, "Unknown command: " + action, "", 400};
    }
}

CommandType CommandHandler::parseCommandType(const String& action) {
    if (action == "arm") return CMD_ARM;
    if (action == "disarm") return CMD_DISARM;
    if (action == "status") return CMD_STATUS;
    if (action == "test_leds") return CMD_TEST_LEDS;
    if (action == "reboot") return CMD_REBOOT;
    return CMD_UNKNOWN;
}

CommandResponse CommandHandler::handleArmCommand(const String& source) {
    if (currentMode == ARMED) {
        return {false, "System is already armed", "", 400};
    }
    
    currentMode = ARMED;
    Serial.println("DEBUG::command_handler.cpp System MANUALLY ARMED from " + source);
    
    // Show blue flashes for manual arm (different from auto-arm red flashes)
    for(int i = 0; i < 3; i++) {
        fill_solid(leds, NUM_LEDS, CRGB::Blue);
        FastLED.show();
        delay(200);
        clearAllLEDs();
        delay(200);
    }
    
    return {true, "System armed successfully", createStatusResponse(), 200};
}

CommandResponse CommandHandler::handleDisarmCommand(const String& source) {
    if (currentMode == DISARMED) {
        return {false, "System is already disarmed", "", 400};
    }
    
    currentMode = DISARMED;
    // Reset auto-arm timer so we don't immediately re-arm after manual disarm
    lastMovementTime = millis();
    Serial.println("DEBUG::command_handler.cpp Resetting auto-arm timer on disarm");
    Serial.println("DEBUG::command_handler.cpp System MANUALLY DISARMED from " + source);
    
    // Show cyan flashes for manual disarm (different from normal green flashes)
    for(int i = 0; i < 3; i++) {
        fill_solid(leds, NUM_LEDS, CRGB::Cyan);
        FastLED.show();
        delay(200);
        clearAllLEDs();
        delay(200);
    }
    
    return {true, "System disarmed successfully", createStatusResponse(), 200};
}

CommandResponse CommandHandler::handleStatusCommand() {
    return {true, "Status retrieved successfully", createStatusResponse(), 200};
}

CommandResponse CommandHandler::handleTestLedsCommand() {
    Serial.println("DEBUG::command_handler.cpp Testing LEDs");
    
    // Test sequence: Red -> Green -> Blue -> White -> Off
    CRGB testColors[] = {CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::White};
    
    for(int color = 0; color < 4; color++) {
        fill_solid(leds, NUM_LEDS, testColors[color]);
        FastLED.show();
        delay(500);
    }
    
    clearAllLEDs();
    
    return {true, "LED test completed", "", 200};
}

CommandResponse CommandHandler::handleRebootCommand() {
    Serial.println("DEBUG::command_handler.cpp Rebooting device in 2 seconds...");
    
    // Flash red 5 times to indicate reboot
    for(int i = 0; i < 5; i++) {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
        FastLED.show();
        delay(200);
        clearAllLEDs();
        delay(200);
    }
    
    delay(2000);
    ESP.restart();
    
    return {true, "Device rebooting", "", 200};
}

String CommandHandler::createStatusResponse() {
    DynamicJsonDocument doc(512);
    
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    mac.toLowerCase();
    doc["device_id"] = mac;
    // Timestamp is handled in MQTT payloads; omit numeric timestamp here
    doc["current_mode"] = (currentMode == ARMED) ? "ARMED" : "DISARMED";
    doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
    doc["wifi_signal"] = WiFi.RSSI();
    doc["uptime_ms"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();
    
    String response;
    serializeJson(doc, response);
    
    return response;
}

bool CommandHandler::validateApiKey(const String& providedKey) {
    return providedKey == API_KEY;
} 