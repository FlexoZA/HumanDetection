# Human Detection Sensor - ESP32 Project

## Overview
This project implements an advanced human detection sensor system using an ESP32 microcontroller with intelligent ARM/DISARM modes. The system features progressive detection stages with visual LED feedback and runs on battery power with WiFi connectivity for notifications.

## Key Features
- **RCWL-0516 Microwave Radar Sensor** - 7-9 meter detection range with 360° coverage
- **Smart ARM/DISARM System** - Automatic arming after 10 minutes of no movement
- **Progressive Detection Stages** - 3-stage loading sequence to eliminate false positives
- **Visual LED Feedback** - 16 LED NeoPixel strip with rainbow and alert effects
- **Repeated Notifications** - Continuous alerts every 5 seconds for sustained detection
- **Battery Optimized** - Low power design for AA battery operation
- **WiFi Integration** - Ready for n8n/Supabase/Telegram notifications

## Hardware Components

### ESP32 Development Board
- Main microcontroller with WiFi capability
- GPIO pins for sensor and LED control
- USB-C power (development) or battery power (deployment)

### RCWL-0516 Microwave Radar Sensor
- **Detection Method**: Doppler shift microwave radar (3.2 GHz)
- **Range**: 7-9 meters with 360° coverage
- **Power**: 4-28V @ 2.8mA (low power consumption)
- **Output**: 3.3V HIGH when motion detected
- **Advantages**: Works through thin walls, not affected by temperature

### CJMCU-2812B-16 NeoPixel LED Strip
- 16 addressable RGB LEDs
- WS2812B protocol with single data line control
- Visual feedback for system status and alerts
- 3.3V compatible with built-in current limiting

## Pin Connections

### RCWL-0516 Sensor Wiring
```
RCWL-0516    ESP32
VIN      →   VIN (5V from USB) or 3.3V
GND      →   GND
OUT      →   GPIO 23
CDS      →   Not connected (optional light sensor)
3V3      →   Not connected
```

### CJMCU-2812B-16 NeoPixel Wiring
```
NeoPixel     ESP32
VCC      →   3.3V
GND      →   GND
DIN      →   GPIO 25
DOUT     →   Not connected
```

### Capacitive Touch Button with 10-Second Safety Delay
```
Touch Button   ESP32 Pin
Capacitive →   GPIO 33 (capacitive touch sensor)
GND Reference → GND (required for capacitive sensing)
```

**🛡️ Safety Feature: Smart 10-Second Delay**
- **To ARM**: Touch + release finger → 10-second countdown begins (progressive purple loading)
- **To DISARM**: Touch + release finger → **immediate** disarm (no delay)
- **Cancel**: Touch again during countdown to cancel arming
- **LED Priority**: Countdown visuals override standby rainbow during arming
- Prevents accidental arming, allows instant disarming

## **Plastic Enclosure Setup:**

**✅ RECOMMENDED - Single Touch (One Finger):**
```
ESP32 GPIO 33 ──── Conductive Pad (touch electrode)
ESP32 GND ──────── Separate Conductive Pad (ground reference)
```
- Touch one conductive pad with one finger
- Your body provides the capacitive connection
- Simple and reliable

**❌ NOT RECOMMENDED - Two Touch (Two Fingers):**
- ESP32 capacitive touch is designed for single-touch detection
- Two-finger detection is unreliable and complex
- Use single touch instead

**❌ WRONG - Single Wire (Shorted):**
```
ESP32 GPIO 33 ──── Same Wire ──── GND (creates short circuit!)
```

### Future Expansion (Planned)
```
Component    ESP32 Pin
Button   →   GPIO 18 (with internal pull-up)
```

## System Operation

### Boot Sequence
1. **Rainbow Chase Effect** - Indicates successful system initialization
2. **System Mode**: DISARMED (continuous rainbow display)
3. **Auto-arm Timer**: Starts 10-minute countdown

### DISARMED Mode (Default)
- **Visual**: Continuous flowing rainbow effect on all LEDs
- **Behavior**: Movement detection disabled for alerts
- **Auto-arm**: Switches to ARMED after 10 minutes of no movement
- **Purpose**: Ambient lighting when residents are present

### ARMED Mode (Security Active)
- **Visual**: LEDs off when idle (stealth mode)
- **Behavior**: Full detection sequence active
- **Auto-disarm**: Currently disabled (requires manual disarm in future)
- **Purpose**: Security monitoring when area should be vacant

### Detection Sequence (ARMED Mode Only)

#### Stage 1: Initial Detection (1 second)
- **Visual**: White LEDs loading progressively (16 LEDs in 1 second)
- **Trigger**: First movement detected
- **Notification**: None (false positive prevention)

#### Stage 2: Continued Presence (1 second)
- **Visual**: Orange LEDs loading progressively
- **Trigger**: Movement continues from Stage 1
- **Notification**: None (still filtering false positives)

#### Stage 3: Sustained Detection (1 second)
- **Visual**: Red LEDs loading progressively
- **Trigger**: Movement continues from Stage 2
- **Notification**: None (final verification stage)

#### Alert State: Confirmed Threat (Continuous)
- **Visual**: Solid red across all 16 LEDs
- **Trigger**: Movement sustained for full 3-second sequence
- **Notifications**: 
  - First notification sent immediately
  - Repeat notifications every 5 seconds until movement stops
- **Duration**: Until movement stops or manual disarm

### Mode Indicators
- **DISARMED Mode**: 2 quick green flashes across all LEDs
- **ARMED Mode**: 2 quick red flashes across all LEDs
- **Notification Sent**: 3 purple flashes to confirm alert transmission
- **Touch Detected**: Quick blue flash followed by mode indicator flashes

## Technical Specifications

### Detection Parameters
- **False Positive Prevention**: 3-second continuous movement requirement
- **Movement Timeout**: 500ms gap considers movement "stopped"
- **Auto-arm Delay**: 10 minutes of no movement (configurable)
- **Notification Interval**: Every 5 seconds during alert state
- **Detection Range**: 7-9 meters (360° coverage)

### LED Effects
- **Rainbow Speed**: Updates every 50ms for smooth animation
- **Loading Speed**: 16 LEDs per second (consistent across all stages)
- **Brightness**: 20% (50/255) for power efficiency and eye comfort
- **Color Accuracy**: Full RGB spectrum with HSV color space

### Power Management
- **Sensor Power**: 2.8mA continuous
- **LED Power**: ~200mA at 20% brightness (all LEDs on)
- **ESP32 Power**: ~80mA active, deep sleep capable
- **Total Active**: ~280mA (suitable for battery operation)

## Software Architecture

### State Machine Design
- **System Modes**: DISARMED ↔ ARMED
- **Detection States**: IDLE → WHITE → ORANGE → RED → ALERT
- **Non-blocking Operation**: All timing uses `millis()` for smooth performance

### Key Functions
- `updateRainbow()` - Continuous rainbow effect for disarmed mode
- `updateLoadingEffect()` - Progressive LED loading for detection stages
- `sendNotification()` - Placeholder for future API integration
- `checkAutoArm()` - Automatic arming logic
- `showModeIndicator()` - Visual mode change feedback

## Development Status

### ✅ Completed Features
- [x] ESP32 and sensor initialization
- [x] RCWL-0516 microwave radar integration
- [x] CJMCU-2812B-16 NeoPixel control
- [x] ARM/DISARM mode system
- [x] Progressive 3-stage detection sequence
- [x] Visual LED feedback (rainbow, loading, alerts)
- [x] False positive prevention (3-second sequence)
- [x] Repeated notification system
- [x] Auto-arming after inactivity
- [x] Mode change indicators
- [x] Capacitive touch button for manual ARM/DISARM

### 🚧 In Development
- [ ] WiFi connectivity and credentials management
- [ ] n8n webhook integration for notifications
- [ ] Supabase database logging
- [ ] Telegram bot alerts
- [ ] Battery voltage monitoring
- [ ] Deep sleep power optimization

### 🔮 Future Enhancements
- [ ] Mobile app for remote control
- [ ] Multiple detection zones
- [ ] Scheduling (auto-arm/disarm times)
- [ ] Motion pattern analysis
- [ ] Integration with home automation systems
- [ ] Over-the-air (OTA) updates

## Configuration

### Timing Constants (Configurable)
```cpp
const unsigned long STAGE1_DURATION = 1000;    // White stage duration
const unsigned long STAGE2_DURATION = 1000;    // Orange stage duration  
const unsigned long STAGE3_DURATION = 1000;    // Red stage duration
const unsigned long AUTO_ARM_DELAY = 600000;   // 10 minutes auto-arm
const unsigned long NOTIFICATION_REPEAT_INTERVAL = 5000; // 5 second repeats
```

### Hardware Pin Definitions
```cpp
#define PIR_SENSOR_PIN 23      // RCWL-0516 output
#define LED_DATA_PIN 25        // NeoPixel data input
#define NUM_LEDS 16           // Number of LEDs in strip
#define BUTTON_PIN 33         // Capacitive touch button
```

### Capacitive Touch Configuration
```cpp
#define TOUCH_THRESHOLD 40     // Touch sensitivity (lower = more sensitive)
#define TOUCH_DEBOUNCE_INTERVAL 2000 // Minimum time between touches (2000ms = 2s)
#define TOUCH_ARM_DELAY 10000  // 10 seconds delay before arming/disarming
```

## Installation & Setup

### Hardware Assembly
1. Connect RCWL-0516 sensor to ESP32 (VIN, GND, OUT → GPIO 23)
2. Connect CJMCU-2812B-16 strip to ESP32 (VCC, GND, DIN → GPIO 25)
3. **Touch Button Wiring (Plastic Enclosure):**
   - **Wire 1**: ESP32 GPIO 33 → Conductive pad/surface (touch electrode)
   - **Wire 2**: ESP32 GND → Separate conductive pad/surface (ground reference)
   - **⚠️ NEVER connect both wires together!**
4. Add conductive material: Copper foil, conductive paint, or metal plates
5. Power ESP32 via USB-C for development/testing

**Grounding Note**: With plastic enclosure, your body becomes part of the capacitive circuit when you touch the conductive pad.

### Software Setup
1. Install PlatformIO IDE or Arduino IDE
2. Clone this repository
3. Install required libraries (FastLED automatically via platformio.ini)
4. Upload code to ESP32
5. Open serial monitor at 115200 baud for debug output

### Testing
1. **Boot Test**: Rainbow chase effect confirms initialization
2. **Disarmed Mode**: Continuous rainbow indicates normal operation
3. **Auto-arm Test**: Wait 10 minutes, system should show red flashes
4. **Detection Test**: Wave hand near sensor, observe 3-stage loading sequence
5. **Alert Test**: Sustain movement for 3+ seconds, confirm solid red and notifications
6. **Ground Test**: Check serial monitor for touch values:
   - **After fixing wiring**: 50-80 when not touching, <40 when touching
   - **If still 0**: GPIO 33 still shorted to GND - check solder joints
   - **If >100**: No ground connection - ensure GND wire is properly connected
   - **If TOUCH DISABLED**: GPIO 33 is still shorted - fix wiring
7. **Touch Calibration**: Monitor serial output for touch values, adjust TOUCH_THRESHOLD if needed
8. **Touch Test**: Test both ARM and DISARM scenarios:
   - **When DISARMED**: Touch + release → blue flash + progressive purple loading (10s countdown)
   - **During countdown**: Touch again → red flash (canceled)
   - **Wait 10s**: Green/red flashes (armed)
   - **When ARMED**: Touch + release → immediate green/red flashes (disarmed)

## Troubleshooting

### Common Issues
- **No LED response**: Check 3.3V power and GPIO 25 connection
- **No motion detection**: Verify RCWL-0516 VIN power and GPIO 23 connection
- **Slow rainbow**: Normal - updates every 50ms for smooth animation
- **No auto-arm**: Check 10-minute timer, ensure no movement detected
- **Missing notifications**: Verify 3-second sustained movement requirement
- **Touch not working**: Check if GPIO 33 is connected to screw AND GND has proper reference
- **Touch values stuck at 0**: GPIO 33 is DIRECTLY connected to GND - remove the direct bridge
- **TOUCH DISABLED message**: GPIO 33 is shorted to GND - fix wiring connections
- **Touch values always high**: No ground reference - ensure GND is connected properly
- **Touch too sensitive/insensitive**: Adjust TOUCH_THRESHOLD value (lower = more sensitive)
- **Touch erratic**: Check GND connection quality and metal surface cleanliness

### Debug Output
Monitor serial output at 115200 baud for detailed system status:
```
DEBUG::main.cpp Starting Advanced Human Detection System with ARM/DISARM...
DEBUG::main.cpp System Mode: DISARMED (continuous rainbow)
DEBUG::main.cpp Touch value: 65 (threshold: 40)  // Normal reading
DEBUG::main.cpp TOUCH: Finger released - Starting 10-second countdown to ARM
DEBUG::main.cpp TOUCH: Touch again to cancel, or wait 10 seconds to arm
DEBUG::main.cpp TOUCH: 10 seconds remaining to arm... [LEDs: 0/16 lit]
DEBUG::main.cpp TOUCH: 9 seconds remaining to arm... [LEDs: 2/16 lit]
[...countdown continues - progressive purple loading...]
DEBUG::main.cpp TOUCH: 10-second delay complete - System now ARMED
DEBUG::main.cpp TOUCH: Finger released - System immediately DISARMED  // When ARMED
DEBUG::main.cpp TOUCH: Countdown canceled - touch detected again  // When canceling
DEBUG::main.cpp TOUCH DISABLED: GPIO 33 shorted to GND - check wiring!  // If shorted
DEBUG::main.cpp STAGE 1: ARMED detection - WHITE loading (no notification yet)
DEBUG::main.cpp ALERT: Full detection sequence complete - SOLID RED + FIRST NOTIFICATION
```

## License
MIT License - See LICENSE file for details

## Contributing
Pull requests welcome! Please test thoroughly and update documentation for any changes.

---

**Project Status**: Active Development  
**Hardware**: ESP32 + RCWL-0516 + CJMCU-2812B-16  
**Power**: USB-C (development) / 4x AA batteries (deployment)  
**Range**: 7-9 meters, 360° coverage  
**Response Time**: 3 seconds (false positive prevention)
