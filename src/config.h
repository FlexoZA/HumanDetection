#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "Lab"
#define WIFI_PASSWORD "jason789"
#define WIFI_TIMEOUT_MS 10000

// Hardware Pin Configuration
// RCWL-0516 Sensor
#define PIR_SENSOR_PIN 23  // GPIO 23 for motion sensor output

// CJMCU-2812B-16 NeoPixel Strip (16 LEDs)
#define LED_DATA_PIN 25    // GPIO 25 for LED data line
#define NUM_LEDS 16        // Number of LEDs in the strip

// Button Pin
#define BUTTON_PIN 18      // GPIO 18 with internal pull-up

// Battery monitoring (if needed later)
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