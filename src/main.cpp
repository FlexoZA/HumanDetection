#include <Arduino.h>
#include <time.h>
#include <FastLED.h>
#include "config.h"
#include "wifi_manager.h"
#include "mqtt_client.h"
#include "mqtt_message_handler.h"
#include "command_handler.h"

// Function declarations for external access
void triggerCommandReceivedIndicator();

// LED array
CRGB leds[NUM_LEDS];
bool bootTestComplete = false;

// System modes
enum SystemMode {
    DISARMED,   // System is disarmed - no alerts, just visual feedback
    ARMED       // System is armed - detection triggers notifications
};

// Human detection state machine
enum DetectionState {
    IDLE,           // No detection
    STAGE1_WHITE,   // First movement detected - white loading
    STAGE2_ORANGE,  // Continued movement - orange loading
    STAGE3_RED      // Sustained movement - red loading with 10 second reset
};

// MQTT connection states for visual feedback
const int MQTT_STATE_DISCONNECTED = 0;
const int MQTT_STATE_CONNECTING = 1;
const int MQTT_STATE_CONNECTED = 2;
const int MQTT_STATE_RECONNECTING = 3;

SystemMode currentMode = DISARMED;  // Start disarmed
DetectionState currentState = IDLE;
int mqttState = MQTT_STATE_DISCONNECTED;
unsigned long stateStartTime = 0;
unsigned long lastMovementTime = 0;
unsigned long lastLEDUpdate = 0;
unsigned long modeChangeTime = 0;
unsigned long lastRainbowUpdate = 0;
unsigned long lastNotificationTime = 0;
unsigned long lastMQTTStatusCheck = 0;
unsigned long lastMQTTVisualUpdate = 0;
uint8_t rainbowHue = 0;
uint8_t mqttPulseHue = 0;
bool lastMQTTConnectedState = false;

// Timing constants (in milliseconds) - Updated for 5 second detection window
const unsigned long STAGE1_DURATION = 1500;    // 1.5 seconds white loading
const unsigned long STAGE2_DURATION = 1500;    // 1.5 seconds orange loading
const unsigned long STAGE3_DURATION = 2000;    // 2 seconds red loading before solid red
const unsigned long MOVEMENT_TIMEOUT = 5000;    // 5000ms (5 seconds) without movement = no movement
// LED timing is handled dynamically by updateLoadingEffect function
const unsigned long AUTO_ARM_DELAY = 600000;   // 10 minutes (600000) to auto-arm after no movement
const unsigned long RAINBOW_UPDATE_INTERVAL = 50; // Update rainbow every 50ms
const unsigned long NOTIFICATION_REPEAT_INTERVAL = 5000; // Send notification every 5 seconds in solid red
const unsigned long MQTT_STATUS_CHECK_INTERVAL = 1000; // Check MQTT status every second
const unsigned long MQTT_VISUAL_UPDATE_INTERVAL = 100; // Update MQTT visual feedback every 100ms

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    
    Serial.println("DEBUG::main.cpp Starting Advanced Human Detection System with ARM/DISARM...");
    
    // Initialize PIR sensor pin
    pinMode(PIR_SENSOR_PIN, INPUT);
    
    // Initialize FastLED
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50); // Set brightness to 50/255 (about 20% - good for testing)
    
    // Turn off all LEDs initially
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    
    Serial.println("DEBUG::main.cpp CJMCU-2812B-16 initialized on pin " + String(LED_DATA_PIN));
    Serial.println("DEBUG::main.cpp RCWL-0516 sensor initialized on pin " + String(PIR_SENSOR_PIN));
    Serial.println("DEBUG::main.cpp Number of LEDs: " + String(NUM_LEDS));
    Serial.println("DEBUG::main.cpp ESP32 boot successful - running rainbow chase");
    
    // Run rainbow chase effect once to indicate successful boot
    for(int i = 0; i < NUM_LEDS; i++) {
        // Clear all LEDs
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        
        // Set current LED to rainbow color
        leds[i] = CHSV(i * (255/NUM_LEDS), 255, 255);
        FastLED.show();
        delay(100);
    }
    
    // Turn off all LEDs after chase
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    
    // Initialize WiFi connection with visual feedback
    Serial.println("DEBUG::main.cpp Initializing WiFi connection...");
    
    // Show blue LEDs while connecting to WiFi
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();
    
    if (WiFiManager::init()) {
        Serial.println("DEBUG::main.cpp WiFi connected successfully!");
        
        // Configure SNTP time
        configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
        Serial.println("DEBUG::main.cpp NTP: starting time sync...");
        
        // Wait up to ~10s for time to sync
        const time_t TIME_SYNC_THRESHOLD = 1600000000; // ~2020-09-13
        int attempts = 0;
        while (attempts < 100) {
            time_t now = time(nullptr);
            if (now > TIME_SYNC_THRESHOLD) {
                Serial.println("DEBUG::main.cpp NTP: time synchronized successfully");
                break;
            }
            delay(100);
            attempts++;
        }
        if (attempts >= 100) {
            Serial.println("DEBUG::main.cpp NTP: time sync timed out, proceeding without RTC time");
        }
        
        // Show green flash for successful WiFi connection
        for(int i = 0; i < 3; i++) {
            fill_solid(leds, NUM_LEDS, CRGB::Green);
            FastLED.show();
            delay(200);
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            delay(200);
        }
        
        // Initialize MQTT client
        MQTTClient::init();
        Serial.println("DEBUG::main.cpp MQTT client initialized");
        
        // Initialize command handler
        CommandHandler::init();
        
        // Initialize MQTT message handler
        MQTTMessageHandler::init();
        
        // Set up MQTT message callback
        MQTTClient::setMessageCallback(MQTTMessageHandler::handleMessage);
        
        // Connect to MQTT broker
        if (MQTTClient::connect()) {
            Serial.println("DEBUG::main.cpp MQTT connected successfully!");
        } else {
            Serial.println("DEBUG::main.cpp MQTT connection failed!");
        }
        
    } else {
        Serial.println("DEBUG::main.cpp WiFi connection failed!");
        
        // Show red flash for failed WiFi connection
        for(int i = 0; i < 5; i++) {
            fill_solid(leds, NUM_LEDS, CRGB::Red);
            FastLED.show();
            delay(300);
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            delay(300);
        }
    }
    
    bootTestComplete = true;
    modeChangeTime = millis(); // Initialize mode change time
    lastRainbowUpdate = millis();
    lastMQTTStatusCheck = millis();
    lastMQTTVisualUpdate = millis();
    lastMQTTConnectedState = MQTTClient::isConnected();
    
    // Initialize lastMovementTime to current time to prevent immediate auto-arm
    lastMovementTime = millis();
    // TODO: Remove debug logging when auto-arm is working correctly - saves memory and serial bandwidth on ESP32
    Serial.println("DEBUG::main.cpp Auto-arm timer initialized - will auto-arm after 15 minutes of no movement");
    
    // Initialize MQTT state based on current connection
    if (WiFiManager::isConnected()) {
        if (MQTTClient::isConnected()) {
            mqttState = MQTT_STATE_CONNECTED;
        } else {
            mqttState = MQTT_STATE_CONNECTING;
        }
    } else {
        mqttState = MQTT_STATE_DISCONNECTED;
    }
    
    Serial.println("DEBUG::main.cpp Boot test complete - System ready");
    Serial.println("DEBUG::main.cpp System Mode: DISARMED (continuous rainbow)");
    Serial.println("DEBUG::main.cpp Auto-arm delay: 15 minutes after no movement");
    Serial.println("DEBUG::main.cpp Detection stages: WHITE loading (1.5s) -> ORANGE loading (1.5s) -> RED loading (2s) -> SOLID RED + MQTT");
    Serial.println("DEBUG::main.cpp MQTT visual feedback enabled");
}

void clearAllLEDs() {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
}

void updateRainbow() {
    unsigned long currentTime = millis();
    
    // Only update rainbow if enough time has passed
    if (currentTime - lastRainbowUpdate >= RAINBOW_UPDATE_INTERVAL) {
        // Create rainbow effect across all LEDs
        for(int i = 0; i < NUM_LEDS; i++) {
            // Each LED gets a different hue based on position and time
            uint8_t hue = rainbowHue + (i * 255 / NUM_LEDS);
            leds[i] = CHSV(hue, 255, 255);
        }
        FastLED.show();
        
        // Increment rainbow hue for next update (creates moving rainbow)
        rainbowHue += 2; // Speed of rainbow movement
        lastRainbowUpdate = currentTime;
    }
}

void showModeIndicator() {
    // Brief mode indicator: 2 quick flashes for current mode
    clearAllLEDs();
    delay(200);
    
    if (currentMode == ARMED) {
        // Armed: 2 quick red flashes
        for(int i = 0; i < 2; i++) {
            fill_solid(leds, NUM_LEDS, CRGB::Red);
            FastLED.show();
            delay(150);
            clearAllLEDs();
            delay(150);
        }
        Serial.println("DEBUG::main.cpp Mode indicator: ARMED (red flashes)");
    } else {
        // Disarmed: 2 quick green flashes  
        for(int i = 0; i < 2; i++) {
            fill_solid(leds, NUM_LEDS, CRGB::Green);
            FastLED.show();
            delay(150);
            clearAllLEDs();
            delay(150);
        }
        Serial.println("DEBUG::main.cpp Mode indicator: DISARMED (green flashes) - resuming rainbow");
    }
}

void showMQTTConnectionPulse() {
    // Blue pulsing effect while connecting to MQTT
    unsigned long currentTime = millis();
    
    if (currentTime - lastMQTTVisualUpdate >= MQTT_VISUAL_UPDATE_INTERVAL) {
        // Create pulsing blue effect
        uint8_t brightness = (sin(mqttPulseHue * 0.02) + 1) * 127; // Sine wave between 0-255
        CRGB pulseColor = CRGB(0, 0, brightness);
        
        fill_solid(leds, NUM_LEDS, pulseColor);
        FastLED.show();
        
        mqttPulseHue += 8; // Speed of pulse
        lastMQTTVisualUpdate = currentTime;
    }
}

void showMQTTDisconnectionWarning() {
    // Orange and red alternating blink for MQTT disconnection
    static bool orangePhase = true;
    static unsigned long lastBlink = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastBlink >= 500) { // Blink every 500ms
        if (orangePhase) {
            fill_solid(leds, NUM_LEDS, CRGB::Orange);
        } else {
            fill_solid(leds, NUM_LEDS, CRGB::Red);
        }
        FastLED.show();
        
        orangePhase = !orangePhase;
        lastBlink = currentTime;
    }
}

void showCommandReceivedIndicator() {
    // Quick white flash to indicate command received
    Serial.println("DEBUG::main.cpp MQTT command received - showing white flash");
    
    // Save current LED state (if needed for restoration)
    // For now, just show the flash and let normal operation resume
    
    // Quick white flash
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    delay(100);
    
    // Brief pause
    clearAllLEDs();
    delay(50);
    
    // Another quick flash
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    delay(100);
    
    // Clear and return to normal operation
    clearAllLEDs();
}

void updateMQTTConnectionState() {
    unsigned long currentTime = millis();
    
    // Check MQTT status periodically
    if (currentTime - lastMQTTStatusCheck >= MQTT_STATUS_CHECK_INTERVAL) {
        bool currentlyConnected = MQTTClient::isConnected();
        
        // Detect state changes
        if (currentlyConnected != lastMQTTConnectedState) {
            if (currentlyConnected) {
                // Just connected
                mqttState = MQTT_STATE_CONNECTED;
                Serial.println("DEBUG::main.cpp MQTT connection established - visual feedback updated");
            } else {
                // Just disconnected
                if (lastMQTTConnectedState) {
                    // Was connected, now disconnected
                    mqttState = MQTT_STATE_RECONNECTING;
                    Serial.println("DEBUG::main.cpp MQTT connection lost - showing disconnection warning");
                } else {
                    // Still disconnected, check if we're trying to connect
                    if (WiFiManager::isConnected()) {
                        mqttState = MQTT_STATE_CONNECTING;
                    } else {
                        mqttState = MQTT_STATE_DISCONNECTED;
                    }
                }
            }
            
            lastMQTTConnectedState = currentlyConnected;
        }
        
        lastMQTTStatusCheck = currentTime;
    }
}

// Global function that can be called from other modules
void triggerCommandReceivedIndicator() {
    showCommandReceivedIndicator();
}

void sendNotification() {
    Serial.println("DEBUG::main.cpp *** SECURITY ALERT *** Armed system detected movement!");
    
    // Try to send MQTT notification
    if (WiFiManager::isConnected() && MQTTClient::isConnected()) {
        Serial.println("DEBUG::main.cpp Sending MQTT detection event...");
        
        if (MQTTClient::publishDetectionEvent("human_detection")) {
            Serial.println("DEBUG::main.cpp MQTT detection event sent successfully!");
        } else {
            Serial.println("DEBUG::main.cpp Failed to send MQTT detection event");
        }
    } else {
        Serial.println("DEBUG::main.cpp Cannot send notification - WiFi or MQTT not connected");
    }
    
    // Visual indication of notification attempt (purple flashes)
    for(int i = 0; i < 3; i++) {
        fill_solid(leds, NUM_LEDS, CRGB::Purple);
        FastLED.show();
        delay(200);
        clearAllLEDs();
        delay(200);
    }
}

void updateLoadingEffect(CRGB color, unsigned long stageDuration) {
    unsigned long currentTime = millis();
    unsigned long timeInStage = currentTime - stateStartTime;
    
    // Calculate how many LEDs should be lit based on time progression
    int ledsToLight = (timeInStage * NUM_LEDS) / stageDuration;
    
    // Clamp to valid range
    if (ledsToLight > NUM_LEDS) ledsToLight = NUM_LEDS;
    if (ledsToLight < 0) ledsToLight = 0;
    
    // Update LEDs only if we need to light more LEDs
    static int lastLedsLit = 0;
    static DetectionState lastState = IDLE;
    
    // Reset LED count if state changed
    if (lastState != currentState) {
        lastLedsLit = 0;
        lastState = currentState;
        clearAllLEDs();
    }
    
    // Light up LEDs progressively
    if (ledsToLight > lastLedsLit) {
        for (int i = lastLedsLit; i < ledsToLight; i++) {
            leds[i] = color;
        }
        FastLED.show();
        lastLedsLit = ledsToLight;
    }
}

bool isMovementDetected() {
    static unsigned long lastChangeTime = 0;
    static bool lastState = false;
    static bool debouncedState = false;
    static const int WINDOW_SIZE = 5;  // Number of samples to average
    static bool readings[WINDOW_SIZE] = {false};  // Circular buffer of readings
    static int readIndex = 0;  // Current position in circular buffer
    static int trueCount = 0;  // Count of TRUE readings in buffer
    
    const unsigned long DEBOUNCE_DELAY = 250; // 250ms debounce time
    const float MOVEMENT_THRESHOLD = 0.6; // 60% of readings must be TRUE to trigger
    
    bool currentReading = digitalRead(PIR_SENSOR_PIN) == HIGH;
    unsigned long currentTime = millis();
    
    // Update moving average
    if (readings[readIndex]) {
        trueCount--; // Remove old TRUE reading
    }
    readings[readIndex] = currentReading;
    if (currentReading) {
        trueCount++; // Add new TRUE reading
    }
    readIndex = (readIndex + 1) % WINDOW_SIZE;
    
    // Calculate current filtered state
    bool filteredReading = ((float)trueCount / WINDOW_SIZE) >= MOVEMENT_THRESHOLD;
    
    // Apply debouncing to filtered reading
    if (filteredReading != lastState) {
        lastChangeTime = currentTime;
        lastState = filteredReading;
    }
    
    // If enough time has passed since the last change
    if ((currentTime - lastChangeTime) >= DEBOUNCE_DELAY) {
        debouncedState = filteredReading;
    }
    
    return debouncedState;
}

void checkAutoArm() {
    unsigned long currentTime = millis();
    
    // Only check for auto-arm if currently disarmed
    if (currentMode == DISARMED) {
        // TODO: Remove debug logging when auto-arm is working correctly - saves memory and serial bandwidth on ESP32
        // Debug: Log timing information every 30 seconds
        static unsigned long lastDebugTime = 0;
        if (currentTime - lastDebugTime >= 30000) { // Every 30 seconds
            unsigned long timeSinceLastMovement = currentTime - lastMovementTime;
            unsigned long remainingTime = AUTO_ARM_DELAY - timeSinceLastMovement;
            Serial.println("DEBUG::main.cpp Auto-arm check - Time since last movement: " + String(timeSinceLastMovement/1000) + "s, Remaining: " + String(remainingTime/1000) + "s");
            lastDebugTime = currentTime;
        }
        
        // Check if enough time has passed since last movement
        if (currentTime - lastMovementTime > AUTO_ARM_DELAY) {
            currentMode = ARMED;
            modeChangeTime = currentTime;
            Serial.println("DEBUG::main.cpp AUTO-ARM: No movement for 15 minutes - System now ARMED");
            
            // Send auto-armed notification via MQTT
            if (WiFiManager::isConnected() && MQTTClient::isConnected()) {
                Serial.println("DEBUG::main.cpp Sending auto-armed event via MQTT...");
                
                if (MQTTClient::publishDetectionEvent("auto_armed")) {
                    Serial.println("DEBUG::main.cpp Auto-armed event sent successfully!");
                } else {
                    Serial.println("DEBUG::main.cpp Failed to send auto-armed event");
                }
            } else {
                Serial.println("DEBUG::main.cpp Cannot send auto-armed event - WiFi or MQTT not connected");
            }
            
            showModeIndicator();
        }
    }
}

void loop() {
    if (!bootTestComplete) {
        return;
    }
    
    unsigned long currentTime = millis();
    bool movementNow = isMovementDetected();
    
    // Track both movement starts and continuous movement
    static bool lastMovementState = false;
    static unsigned long lastActiveMovement = 0;  // Tracks the last time we saw any movement
    
    if (movementNow) {
        // Update for any movement (start or continuous)
        lastActiveMovement = currentTime;
        
        // Log only movement starts (for auto-arm timing)
        if (!lastMovementState) {
            lastMovementTime = currentTime;
            Serial.println("DEBUG::main.cpp Movement detected - Auto-arm timer reset to 15 minutes");
        }
    }
    lastMovementState = movementNow;
    
    // Movement is active if we've seen any movement within MOVEMENT_TIMEOUT
    bool movementActive = (currentTime - lastActiveMovement) < MOVEMENT_TIMEOUT;
    
    // Check for auto-arm condition
    checkAutoArm();
    
    // Update MQTT connection state and visual feedback
    updateMQTTConnectionState();
    
    // Handle MQTT visual feedback - only when not in active detection sequence
    bool inActiveDetection = (currentMode == ARMED && currentState != IDLE);
    
    if (!inActiveDetection) {
        switch (mqttState) {
            case MQTT_STATE_CONNECTING:
                showMQTTConnectionPulse();
                break;
            case MQTT_STATE_RECONNECTING:
                showMQTTDisconnectionWarning();
                break;
            case MQTT_STATE_CONNECTED:
                // Normal operation - will show rainbow in disarmed mode or stay dark in armed mode
                break;
            case MQTT_STATE_DISCONNECTED:
                // Show disconnection warning if WiFi is connected but MQTT isn't
                if (WiFiManager::isConnected()) {
                    showMQTTDisconnectionWarning();
                }
                break;
        }
    }
    
    // Only process detection states if system is ARMED
    if (currentMode == ARMED) {
        // State machine for progressive detection (ARMED mode only)
        switch (currentState) {
            case IDLE:
                // Armed mode - keep LEDs off when idle
                clearAllLEDs();
                
                if (movementNow) {
                    // First movement detected - enter STAGE1
                    currentState = STAGE1_WHITE;
                    stateStartTime = currentTime;
                    clearAllLEDs();
                    Serial.println("DEBUG::main.cpp STAGE 1: ARMED detection - WHITE loading 1.5s (no notification yet)");
                    // NOTE: No notification here - wait for full sequence to complete
                }
                break;
                
            case STAGE1_WHITE:
                updateLoadingEffect(CRGB::White, STAGE1_DURATION);
                
                // Check state duration and movement status
                if (currentTime - stateStartTime >= STAGE1_DURATION) {
                    if (movementActive) {
                        // Movement is still active after duration - progress to STAGE2
                        currentState = STAGE2_ORANGE;
                        stateStartTime = currentTime;
                        clearAllLEDs();
                        Serial.println("DEBUG::main.cpp STAGE 2: Continued movement - ORANGE loading 1.5s");
                    }
                } else if (!movementActive) {
                    // Movement stopped before completing stage - return to idle
                    currentState = IDLE;
                    clearAllLEDs();
                    Serial.println("DEBUG::main.cpp Movement stopped in WHITE stage - returning to IDLE (no notification sent)");
                }
                break;
                
            case STAGE2_ORANGE:
                updateLoadingEffect(CRGB::Orange, STAGE2_DURATION);
                
                // Check state duration and movement status
                if (currentTime - stateStartTime >= STAGE2_DURATION) {
                    if (movementActive) {
                        // Movement is still active after duration - progress to STAGE3
                        currentState = STAGE3_RED;
                        stateStartTime = currentTime;
                        clearAllLEDs();
                        Serial.println("DEBUG::main.cpp STAGE 3: Sustained movement - RED loading 2s");
                    }
                } else if (!movementActive) {
                    // Movement stopped before completing stage - return to idle
                    currentState = IDLE;
                    clearAllLEDs();
                    Serial.println("DEBUG::main.cpp Movement stopped in ORANGE stage - returning to IDLE (no notification sent)");
                }
                break;
                
            case STAGE3_RED:
                if (currentTime - stateStartTime < STAGE3_DURATION) {
                    // Still in loading phase
                    updateLoadingEffect(CRGB::Red, STAGE3_DURATION);
                } else {
                    // Loading complete - show solid red and send repeated notifications
                    fill_solid(leds, NUM_LEDS, CRGB::Red);
                    FastLED.show();
                    
                    // Send first notification immediately when entering solid red
                    if (lastNotificationTime == 0) {
                        Serial.println("DEBUG::main.cpp ALERT: 5-second detection sequence complete - SOLID RED + FIRST MQTT NOTIFICATION");
                        sendNotification();
                        lastNotificationTime = currentTime;
                    }
                    // Send repeated notifications every 5 seconds
                    else if (currentTime - lastNotificationTime >= NOTIFICATION_REPEAT_INTERVAL) {
                        Serial.println("DEBUG::main.cpp ALERT: Continued detection - SOLID RED + REPEAT NOTIFICATION");
                        sendNotification();
                        lastNotificationTime = currentTime;
                    }
                }
                
                if (!movementActive) {
                    // Movement stopped - return to idle and reset notification timer
                    currentState = IDLE;
                    lastNotificationTime = 0; // Reset for next detection sequence
                    clearAllLEDs();
                    Serial.println("DEBUG::main.cpp Movement stopped in RED stage - returning to IDLE");
                }
                break;
        }
    } else {
        // DISARMED mode - show rainbow only if MQTT is connected, otherwise show MQTT status
        currentState = IDLE; // Always stay in idle when disarmed
        
        if (mqttState == MQTT_STATE_CONNECTED) {
            updateRainbow();
        }
        // MQTT visual feedback is handled above in the main switch statement
    }
    
    // Handle MQTT client loop (includes message handling and reconnection)
    MQTTClient::loop();
    
    // Small delay to prevent excessive CPU usage
    delay(20);
} 