#include "power_manager.h"
#include "config.h"

// Static member initialization
bool PowerManager::initialized = false;
unsigned long PowerManager::bootTime = 0;
float PowerManager::lastBatteryVoltage = 0.0;
unsigned long PowerManager::lastVoltageCheck = 0;

void PowerManager::init() {
    Serial.println("DEBUG::power_manager.cpp Initializing power management...");
    
    bootTime = millis();
    
    // Configure ADC for battery voltage reading
    analogReadResolution(12); // 12-bit ADC resolution
    analogSetAttenuation(ADC_11db); // For 3.3V range
    
    // Initial battery voltage reading
    lastBatteryVoltage = readBatteryVoltage();
    lastVoltageCheck = millis();
    
    // Configure wakeup sources
    configureWakeupSources();
    
    // Optimize for battery usage
    optimizeForBattery();
    
    initialized = true;
    
    Serial.println("DEBUG::power_manager.cpp Power management initialized");
    Serial.println("DEBUG::power_manager.cpp Initial battery voltage: " + String(lastBatteryVoltage) + "V");
}

float PowerManager::getBatteryVoltage() {
    if (!initialized) {
        return 0.0;
    }
    
    // Update voltage reading every 10 seconds to avoid excessive ADC reads
    if (millis() - lastVoltageCheck > 10000) {
        lastBatteryVoltage = readBatteryVoltage();
        lastVoltageCheck = millis();
    }
    
    return lastBatteryVoltage;
}

int PowerManager::getBatteryPercentage() {
    float voltage = getBatteryVoltage();
    
    // Convert voltage to percentage (rough estimate for 4x AA batteries)
    // 4x AA: 6.0V full, 4.8V empty
    float percentage = ((voltage - 4.8) / (6.0 - 4.8)) * 100;
    
    // Clamp to 0-100 range
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;
    
    return (int)percentage;
}

bool PowerManager::isLowBattery() {
    return getBatteryVoltage() < LOW_BATTERY_THRESHOLD;
}

void PowerManager::enterDeepSleep(int seconds) {
    Serial.println("DEBUG::power_manager.cpp Entering deep sleep for " + String(seconds) + " seconds");
    Serial.flush(); // Ensure all serial output is sent
    
    // Configure timer wakeup
    esp_sleep_enable_timer_wakeup(seconds * 1000000ULL); // Convert to microseconds
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

void PowerManager::enterLightSleep(int seconds) {
    Serial.println("DEBUG::power_manager.cpp Entering light sleep for " + String(seconds) + " seconds");
    Serial.flush();
    
    // Configure timer wakeup
    esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
    
    // Enter light sleep
    esp_light_sleep_start();
    
    Serial.println("DEBUG::power_manager.cpp Woke up from light sleep");
}

void PowerManager::optimizeForBattery() {
    Serial.println("DEBUG::power_manager.cpp Optimizing for battery usage...");
    
    // Reduce CPU frequency
    setCpuFrequencyMhz(80); // Reduce from 240MHz to 80MHz
    
    // Disable unused peripherals
    reducePowerConsumption();
    
    Serial.println("DEBUG::power_manager.cpp Battery optimization complete");
}

unsigned long PowerManager::getUptime() {
    if (!initialized) {
        return 0;
    }
    
    return millis() - bootTime;
}

float PowerManager::readBatteryVoltage() {
    // Read ADC value
    int adcValue = analogRead(BATTERY_VOLTAGE_PIN);
    
    // Convert to voltage (assuming 3.3V reference and voltage divider)
    float voltage = (adcValue / 4095.0) * 3.3 * BATTERY_VOLTAGE_DIVIDER;
    
    return voltage;
}

void PowerManager::configureWakeupSources() {
    Serial.println("DEBUG::power_manager.cpp Configuring wakeup sources...");
    
    // Configure PIR sensor as wakeup source
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 1); // Wake on HIGH (motion detected)
    
    // Configure timer wakeup as backup
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SECONDS * 1000000ULL);
    
    Serial.println("DEBUG::power_manager.cpp Wakeup sources configured");
}

void PowerManager::reducePowerConsumption() {
    // Disable Bluetooth
    btStop();
    
    // Additional power optimizations can be added here
    Serial.println("DEBUG::power_manager.cpp Power consumption reduced");
} 