# Human Detection MQTT Implementation TODO

## 🎯 Project Goal
Replace HTTP polling with MQTT for real-time bidirectional communication between ESP32 Human Detection Sensor and N8N using self-hosted Mosquitto broker.

## 📋 Implementation Tasks

### Phase 1: Environment Setup
- [ ] **1.1** Create new git branch `feature/mqtt_integration`
- [ ] **1.2** Verify Mosquitto broker is running and accessible
- [ ] **1.3** Test Mosquitto broker connectivity (mosquitto_pub/sub)
- [ ] **1.4** Document broker connection details (host, port, credentials)

### Phase 2: ESP32 MQTT Configuration
- [ ] **2.1** Update `platformio.ini` to include PubSubClient library
- [ ] **2.2** Update `src/config.h` with MQTT broker settings
- [ ] **2.3** Define MQTT topic structure and message formats
- [ ] **2.4** Add MQTT timing constants (reconnect, heartbeat intervals)

### Phase 3: MQTT Client Implementation
- [ ] **3.1** Create `src/mqtt_client.h` - MQTT client interface
- [ ] **3.2** Create `src/mqtt_client.cpp` - MQTT client implementation
- [ ] **3.3** Implement MQTT connection with auto-reconnect
- [ ] **3.4** Implement message publishing methods
- [ ] **3.5** Implement message subscription and callback handling
- [ ] **3.6** Add connection status monitoring

### Phase 4: Message Handlers
- [ ] **4.1** Create command message parser
- [ ] **4.2** Integrate existing `CommandHandler` with MQTT
- [ ] **4.3** Implement detection event publishing
- [ ] **4.4** Implement auto-armed event publishing
- [ ] **4.5** Implement status/heartbeat publishing
- [ ] **4.6** Add message validation and error handling

### Phase 5: Main Application Integration
- [ ] **5.1** Update `src/main.cpp` includes for MQTT
- [ ] **5.2** Initialize MQTT client in setup()
- [ ] **5.3** Add MQTT client loop handling in main loop
- [ ] **5.4** Replace webhook calls with MQTT publishing
- [ ] **5.5** Add MQTT connection status LED indicators
- [ ] **5.6** Update debug logging for MQTT events

### Phase 6: Visual Feedback System
- [ ] **6.1** Add MQTT connection status LED (blue pulse when connected)
- [ ] **6.2** Add MQTT disconnection warning (red blink)
- [ ] **6.3** Add command received indicator (white flash)
- [ ] **6.4** Maintain existing detection LED sequences
- [ ] **6.5** Update mode indicators for MQTT commands

### Phase 7: N8N Workflow Implementation
- [ ] **7.1** Create N8N MQTT connection to Mosquitto broker
- [ ] **7.2** Create N8N workflow to subscribe to detection events
- [ ] **7.3** Create N8N workflow to subscribe to status/heartbeat
- [ ] **7.4** Create N8N workflow to publish commands
- [ ] **7.5** Test N8N MQTT In/Out nodes functionality
- [ ] **7.6** Implement error handling in N8N workflows

### Phase 8: Testing & Validation
- [ ] **8.1** Test MQTT broker connection from ESP32
- [ ] **8.2** Test command publishing from N8N to ESP32
- [ ] **8.3** Test detection event publishing from ESP32 to N8N
- [ ] **8.4** Test auto-armed event publishing
- [ ] **8.5** Test MQTT reconnection after network interruption
- [ ] **8.6** Test system behavior during broker downtime
- [ ] **8.7** Validate message delivery with QoS settings

### Phase 9: Documentation & Cleanup
- [ ] **9.1** Update README.md with MQTT implementation details
- [ ] **9.2** Document MQTT topic structure and message formats
- [ ] **9.3** Create N8N workflow export/import instructions
- [ ] **9.4** Add troubleshooting guide for MQTT issues
- [ ] **9.5** Remove old HTTP polling code
- [ ] **9.6** Clean up unused dependencies

## 📡 MQTT Architecture

### Topic Structure
```
humandetection/
├── device_[MAC_ADDRESS]/
│   ├── command          # N8N → ESP32 (arm, disarm, test_leds, reboot)
│   ├── status           # ESP32 → N8N (system status, command responses)
│   ├── detection        # ESP32 → N8N (motion alerts)
│   ├── heartbeat        # ESP32 → N8N (keep-alive every 5 minutes)
│   └── online           # ESP32 → N8N (LWT: online/offline)
```

### Message Formats

#### Command Messages (N8N → ESP32)
```json
{
  "action": "arm|disarm|test_leds|reboot|status",
  "source": "n8n_manual|n8n_schedule|n8n_automation",
  "timestamp": 1234567890,
  "api_key": "HumanDetection2024"
}
```

#### Detection Events (ESP32 → N8N)
```json
{
  "event_type": "human_detection|auto_armed",
  "device_id": "MAC_ADDRESS",
  "timestamp": 1234567890,
  "battery_voltage": 3.7,
  "wifi_signal": -45,
  "current_mode": "ARMED|DISARMED",
  "location": "sensor_location"
}
```

#### Status Messages (ESP32 → N8N)
```json
{
  "device_id": "MAC_ADDRESS",
  "current_mode": "ARMED|DISARMED", 
  "wifi_connected": true,
  "mqtt_connected": true,
  "uptime_ms": 123456,
  "free_heap": 200000,
  "last_command": "arm",
  "last_command_source": "n8n_manual",
  "timestamp": 1234567890
}
```

## ⚙️ Configuration Requirements

### Mosquitto Broker Settings
- **Host**: `mqtt.cwe.cloud`
- **Port**: `1883 (MQTT) or 9001 (WebSocket)`
- **Username**: `flexo`
- **Password**: `Chr0846400936#`
- **Client ID**: `humandetection_device_[MAC]`

### QoS Levels
- **Commands**: QoS 1 (at least once delivery)
- **Detection Events**: QoS 1 (critical alerts)
- **Status/Heartbeat**: QoS 0 (fire and forget)

### Timing Configuration
- **Heartbeat Interval**: 300 seconds (5 minutes)
- **Reconnect Interval**: 5 seconds
- **Keep Alive**: 60 seconds
- **Connection Timeout**: 10 seconds

## 🔄 Communication Flow

1. **ESP32 Startup**:
   - Connect WiFi → Connect MQTT → Subscribe to command topic
   - Publish online status with retained flag

2. **Command Execution**:
   - N8N publishes command → ESP32 receives instantly → Executes → Publishes result

3. **Motion Detection**:
   - ESP32 detects motion → Publishes detection event → N8N receives → Triggers automation

4. **System Monitoring**:
   - ESP32 publishes heartbeat every 5 minutes → N8N monitors device health

## 🎯 Success Criteria

- [ ] Commands execute within 100ms of N8N publishing
- [ ] Detection events reach N8N within 200ms
- [ ] System automatically reconnects after network/broker outages  
- [ ] All existing LED feedback and detection logic preserved
- [ ] N8N can reliably control and monitor the device
- [ ] Clean, maintainable code with proper error handling

## 📝 Notes

- Keep existing WiFi manager and sensor logic unchanged
- Maintain backward compatibility with existing detection sequences
- Add comprehensive debug logging for troubleshooting
- Consider implementing TLS encryption for production deployment
- Plan for multiple device support in topic structure

---

**Branch**: `feature/mqtt_integration`  
**Target Completion**: [Date]  
**Dependencies**: Self-hosted Mosquitto broker, N8N MQTT nodes
