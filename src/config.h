#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "Lab"
#define WIFI_PASSWORD "jason789"
#define WIFI_TIMEOUT_MS 10000

// MQTT Configuration
#define MQTT_BROKER "185.202.223.35"
#define MQTT_PORT 1883
#define MQTT_USERNAME "flexo"
#define MQTT_PASSWORD "Chr0846400936#"
#define MQTT_CLIENT_ID_PREFIX "humandetection_device_"
#define MQTT_KEEP_ALIVE 60
#define MQTT_CONNECT_TIMEOUT 10000
#define MQTT_RECONNECT_INTERVAL 5000
#define MQTT_HEARTBEAT_INTERVAL 300000  // 5 minutes

// NTP Configuration
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define NTP_SERVER_3 "time.google.com"

// MQTT Topics
#define MQTT_TOPIC_BASE "humandetection/"
#define MQTT_TOPIC_COMMAND "/command"
#define MQTT_TOPIC_STATUS "/status"
#define MQTT_TOPIC_DETECTION "/detection"
#define MQTT_TOPIC_HEARTBEAT "/heartbeat"
#define MQTT_TOPIC_ONLINE "/online"

// MQTT QoS Levels
#define MQTT_QOS_COMMANDS 1     // At least once delivery for commands
#define MQTT_QOS_DETECTION 1    // At least once delivery for critical alerts
#define MQTT_QOS_STATUS 0       // Fire and forget for status/heartbeat

// Hardware Pin Configuration
// RCWL-0516 Sensor
#define PIR_SENSOR_PIN 23  // GPIO 25 for motion sensor output

// CJMCU-2812B-16 NeoPixel Strip (16 LEDs)
#define LED_DATA_PIN 25    // GPIO 23 for LED data line
#define NUM_LEDS 16        // Number of LEDs in the strip

// Button Pin
#define BUTTON_PIN 18      // GPIO 18 with internal pull-up

// Battery monitoring (if needed later)
#define BATTERY_VOLTAGE_PIN A0

// API Configuration (Legacy - will be removed)
#define API_KEY "HumanDetection2024"

// HTTP Server Configuration (for local commands if needed)
#define HTTP_SERVER_PORT 80
#define MAX_COMMAND_LENGTH 512
#define HTTP_REQUEST_TIMEOUT 5000

// Power Management
#define SLEEP_DURATION_SECONDS 30
#define LOW_BATTERY_THRESHOLD 3.2  // Volts
#define BATTERY_VOLTAGE_DIVIDER 2.0  // Adjust based on your voltage divider

// Sensor Configuration
#define DETECTION_COOLDOWN_MS 5000  // Minimum time between detections
#define PIR_STABILIZATION_TIME_MS 2000  // Time to wait for PIR to stabilize

// Debug Configuration
#define DEBUG_MODE true
#define SERIAL_BAUD_RATE 115200

#endif // CONFIG_H 