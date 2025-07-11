#include "sensor_manager.h"
#include "config.h"

// Static member initialization
bool SensorManager::sensorInitialized = false;
unsigned long SensorManager::initializationTime = 0;
int SensorManager::detectionCount = 0;
bool SensorManager::lastSensorState = false;
unsigned long SensorManager::lastStateChangeTime = 0;

void SensorManager::init() {
    Serial.println("DEBUG::sensor_manager.cpp Initializing sensor...");
    
    pinMode(PIR_SENSOR_PIN, INPUT);
    
    initializationTime = millis();
    sensorInitialized = true;
    lastSensorState = false;
    lastStateChangeTime = millis();
    
    Serial.println("DEBUG::sensor_manager.cpp Sensor initialized, waiting for stabilization...");
    
    // Wait for PIR sensor to stabilize
    delay(PIR_STABILIZATION_TIME_MS);
    
    Serial.println("DEBUG::sensor_manager.cpp Sensor stabilization complete");
}

bool SensorManager::isHumanDetected() {
    if (!sensorInitialized || !isStabilized()) {
        return false;
    }
    
    bool currentState = readSensor();
    
    // Detect rising edge (motion detected)
    if (currentState && !lastSensorState) {
        Serial.println("DEBUG::sensor_manager.cpp Motion detected!");
        updateDetectionCount();
        lastStateChangeTime = millis();
        lastSensorState = currentState;
        return true;
    }
    
    lastSensorState = currentState;
    return false;
}

void SensorManager::calibrate() {
    Serial.println("DEBUG::sensor_manager.cpp Calibrating sensor...");
    
    // Reset calibration
    initializationTime = millis();
    lastSensorState = false;
    
    // Wait for stabilization
    delay(PIR_STABILIZATION_TIME_MS);
    
    Serial.println("DEBUG::sensor_manager.cpp Sensor calibration complete");
}

bool SensorManager::isStabilized() {
    if (!sensorInitialized) {
        return false;
    }
    
    return (millis() - initializationTime) >= PIR_STABILIZATION_TIME_MS;
}

int SensorManager::getDetectionCount() {
    return detectionCount;
}

void SensorManager::resetDetectionCount() {
    detectionCount = 0;
    Serial.println("DEBUG::sensor_manager.cpp Detection count reset");
}

bool SensorManager::readSensor() {
    return digitalRead(PIR_SENSOR_PIN) == HIGH;
}

void SensorManager::updateDetectionCount() {
    detectionCount++;
    Serial.println("DEBUG::sensor_manager.cpp Detection count: " + String(detectionCount));
} 