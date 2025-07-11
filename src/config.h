#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define WIFI_TIMEOUT_MS 10000

// Hardware Pin Configuration
#define PIR_SENSOR_PIN 2
#define LED_PIN 2
#define BATTERY_VOLTAGE_PIN A0

// API Configuration
#define N8N_ENDPOINT_URL "https://your-n8n-instance.com/webhook/human-detection"
#define API_TIMEOUT_MS 5000

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