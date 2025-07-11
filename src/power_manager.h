#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include <esp_sleep.h>

class PowerManager {
public:
    static void init();
    static float getBatteryVoltage();
    static int getBatteryPercentage();
    static bool isLowBattery();
    static void enterDeepSleep(int seconds);
    static void enterLightSleep(int seconds);
    static void optimizeForBattery();
    static unsigned long getUptime();
    
private:
    static bool initialized;
    static unsigned long bootTime;
    static float lastBatteryVoltage;
    static unsigned long lastVoltageCheck;
    
    static float readBatteryVoltage();
    static void configureWakeupSources();
    static void reducePowerConsumption();
};

#endif // POWER_MANAGER_H 