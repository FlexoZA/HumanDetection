# Human Detection Sensor - ESP32 Project

## Overview
This project implements a human detection sensor system using an ESP32 microcontroller. The system runs on AA batteries and communicates via WiFi to send detection events to an n8n endpoint, which then stores data in Supabase and sends notifications via Telegram.

## Architecture
- **Hardware**: ESP32 microcontroller with human detection sensor (PIR/radar)
- **Power**: AA battery powered with low-power optimization
- **Communication**: WiFi connectivity
- **Backend**: n8n workflow automation
- **Database**: Supabase
- **Notifications**: Telegram bot

## Project Structure
```
HumanDetection/
├── src/
│   ├── main.cpp              # Main application entry point
│   ├── config.h              # Configuration constants
│   ├── wifi_manager.h/cpp    # WiFi connection management
│   ├── sensor_manager.h/cpp  # Human detection sensor handling
│   ├── api_client.h/cpp      # HTTP client for n8n communication
│   └── power_manager.h/cpp   # Battery and power management
├── lib/                      # External libraries
├── include/                  # Header files
├── platformio.ini            # PlatformIO configuration
└── README.md                 # This file
```

## Hardware Requirements
- ESP32 development board
- PIR motion sensor or radar sensor
- AA battery pack (4x AA recommended)
- Breadboard and jumper wires
- Pull-up resistors (if needed)

## Software Requirements
- PlatformIO IDE or Arduino IDE
- WiFi network credentials
- n8n workflow endpoint URL
- Supabase project setup
- Telegram bot token

## Setup Instructions
1. Clone this repository
2. Install PlatformIO or Arduino IDE
3. Configure WiFi credentials in `src/config.h`
4. Set up n8n workflow endpoint
5. Configure Supabase database
6. Set up Telegram bot
7. Upload code to ESP32

## Configuration
Update `src/config.h` with your specific settings:
- WiFi credentials
- n8n endpoint URL
- Sensor pin configurations
- Power management settings

## Development Status
- [x] Project structure setup
- [x] Basic C++ file structure created
- [x] WiFi connection management (basic implementation)
- [x] Sensor integration (PIR sensor support)
- [x] API communication (HTTP client for n8n)
- [x] Power management (battery monitoring & deep sleep)
- [ ] Testing and optimization
- [ ] Hardware integration testing
- [ ] n8n workflow setup
- [ ] Supabase integration
- [ ] Telegram bot configuration

## License
MIT License
