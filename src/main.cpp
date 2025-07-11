#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "wifi_manager.h"
#include "sensor_manager.h"
#include "api_client.h"
#include "power_manager.h"

// Global variables
unsigned long lastDetectionTime = 0;
bool systemInitialized = false;

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    
    Serial.println("DEBUG::main.cpp Starting Human Detection System...");
    
    // Initialize power management
    PowerManager::init();
    
    // Check battery level
    float batteryVoltage = PowerManager::getBatteryVoltage();
    Serial.println("DEBUG::main.cpp Battery voltage: " + String(batteryVoltage) + "V");
    
    if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
        Serial.println("DEBUG::main.cpp Low battery detected, entering deep sleep");
        PowerManager::enterDeepSleep(SLEEP_DURATION_SECONDS * 10); // Sleep longer on low battery
    }
    
    // Initialize WiFi
    if (!WiFiManager::init()) {
        Serial.println("DEBUG::main.cpp WiFi initialization failed, entering deep sleep");
        PowerManager::enterDeepSleep(SLEEP_DURATION_SECONDS);
    }
    
    // Initialize sensor
    SensorManager::init();
    
    // Initialize API client
    APIClient::init();
    
    Serial.println("DEBUG::main.cpp System initialization complete");
    systemInitialized = true;
}

void loop() {
    if (!systemInitialized) {
        return;
    }
    
    // Check for human detection
    if (SensorManager::isHumanDetected()) {
        unsigned long currentTime = millis();
        
        // Check if enough time has passed since last detection (debounce)
        if (currentTime - lastDetectionTime > DETECTION_COOLDOWN_MS) {
            Serial.println("DEBUG::main.cpp Human detected!");
            
            // Send detection event to n8n
            if (APIClient::sendDetectionEvent()) {
                Serial.println("DEBUG::main.cpp Detection event sent successfully");
            } else {
                Serial.println("DEBUG::main.cpp Failed to send detection event");
            }
            
            lastDetectionTime = currentTime;
        }
    }
    
    // Check if we should enter sleep mode
    static unsigned long lastActivityTime = millis();
    if (millis() - lastActivityTime > (SLEEP_DURATION_SECONDS * 1000)) {
        Serial.println("DEBUG::main.cpp Entering deep sleep mode");
        PowerManager::enterDeepSleep(SLEEP_DURATION_SECONDS);
    }
    
    // Small delay to prevent excessive CPU usage
    delay(100);
} 