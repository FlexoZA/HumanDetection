#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

class SensorManager {
public:
    static void init();
    static bool isHumanDetected();
    static void calibrate();
    static bool isStabilized();
    static int getDetectionCount();
    static void resetDetectionCount();
    
private:
    static bool sensorInitialized;
    static unsigned long initializationTime;
    static int detectionCount;
    static bool lastSensorState;
    static unsigned long lastStateChangeTime;
    
    static bool readSensor();
    static void updateDetectionCount();
};

#endif // SENSOR_MANAGER_H 