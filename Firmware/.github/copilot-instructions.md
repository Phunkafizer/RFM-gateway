# RFM Gateway Firmware - AI Agent Instructions

## Project Overview
An ESP8266-based IoT gateway that receives and transmits RF signals using RFM69/RFM95/96/97/98 radio modules. The firmware acts as a bridge between RF protocols (868/433 MHz, FS20, RC433, Intertechno) and MQTT/HTTP, allowing Home Automation control. Architecture: async web server + multi-application plugin system + radio abstraction layer.

## Architecture & Core Components

### Multi-Application Plugin System
- **Base class**: `RadioApplication` ([src/radioapplication.h](src/radioapplication.h)) - inherit from this for new radio protocols
- **Active applications**: 
  - `Gw868` ([src/applications/868gw.h](src/applications/868gw.h)) - 868MHz gateway
  - `Fs20` ([src/applications/fs20.h](src/applications/fs20.h)) - FS20 protocol
  - `Rc433` ([src/applications/rc433.h](src/applications/rc433.h)) - RC433 remote codes
- **Key methods**: `loop()` (called every cycle), `onMqttMessage()` (MQTT message handler)
- **Critical**: Global `radioapp` pointer switched at runtime based on config; ensure cleanup in destructors

### Radio Hardware Abstraction
- **Base**: `RfmBase` ([include/rfm.h](include/rfm.h)) - SPI register read/write primitives
- **Implementation**: `Rfm69` - handles mode switching (sleep/standby/TX/RX), frequency correction, power levels
- **Key constants**: FXOSC=32MHz, FSTEP=0.061Hz frequency granularity
- **Register access patterns**: Read reg → modify bits → write reg (use `setReg()` for atomic bit manipulation)
- **Critical signal**: RSSI (Received Signal Strength Indicator) cached on read

### System Services
- **Web server**: `AsyncWebServer` on port 80 - serves UI ([data/index.html](data/index.html) embedded as C string in [include/html.h](include/html.h))
- **WebSocket**: `/ws` endpoint for live data updates
- **MQTT**: `PubSubClient` - topic: `${baseTopic}/...`, loaded from `config.json`
- **DNS**: Port 53 for AP mode hostname resolution
- **LED**: Ticker-based blink pattern (mask: 0x8000 >> count) on `LED_BUILTIN`

## Build & Configuration

### PlatformIO Environments
- **Board**: ESP12E (ESP8266), LittleFS filesystem, 921600 baud upload
- **Compiler**: C++17, Wall/Wextra warnings
- **Libraries**: ArduinoJson, PubSubClient, ESPAsyncWebServer, NTPClient
- **Command**: `pio run -e debug` (with debug logs) or `-e release`
- **Build hook**: [helper.py](helper.py) pre-builds `copy_html()` to compile [data/index.html](data/index.html) → [include/html.h](include/html.h) as PROGMEM constant

### Configuration Files
- `radio.json` - RFM type (0-5), frequency correction, loaded by `loadRadioSetup()`
- `config.json` - MQTT credentials, AP name (default "RFM-Gateway"), WiFi SSID/PSK
- **Default AP**: IP 4.3.2.1, PSK "12345678"

## Critical Patterns & Conventions

### JSON Handling
- **All config I/O**: Uses `ArduinoJson` `JsonDocument`; always check `deserializeJson()` return for `DeserializationError::Ok`
- **Strings**: Heavy PROGMEM usage (const char[] FPSTR macros) to save RAM on ESP8266

### Async Event Loop
- **Main loop**: [src/main.cpp](src/main.cpp) calls `radioapp->loop()` continuously
- **Blocking forbidden**: Use `AsyncWebServer` handlers, never `delay()` in `loop()`
- **Ticker callbacks**: Non-blocking for LED/timers - see `ledTickcb()`

### Frequency Correction
- `setFCorr()` in Rfm69 adjusts oscillator offset (common ±100 ppm drift)
- Store in `radio.json`, exposed via API for tuning

### Web API Endpoints
- `POST /txtest` - raw transmit test (testrfm.py client: [testrfm.py](testrfm.py))
- `GET /send/{protocol}/{params}` - routed to active application's protocol handler
- WebSocket `/ws` - bidirectional app-specific messages

## Common Tasks

### Adding a New Radio Protocol
1. Create `src/applications/newprotocol.h/cpp` inheriting `RadioApplication`
2. Implement `loop()` (called ~100Hz) and `onMqttMessage()` if MQTT control needed
3. Use global `rfm69` to TX/RX; access `baseTopic` for MQTT paths
4. Register in `main.cpp` application switch (update RFM type enum if adding hardware variant)
5. Add `rcpulse.cpp`/`rccodecs.cpp` utilities for modulation if needed

### Debugging
- Enable `-D DEBUG` in platformio.ini debug env for Serial logs
- `monitor_speed = 76800` (check baud in Serial Monitor)
- Exception decoder enabled in debug env for crash analysis
- Use `testrfm.py` to interactively send RF commands to gateway (HTTP at 4.3.2.1)

### Memory Constraints
- ESP8266 has ~80KB free heap - all configs must use PROGMEM strings
- RFM FIFO is 66 bytes max; multi-part messages require framing
- LittleFS overhead; keep JSON configs minimal

## File Organization
- `src/` - Core firmware (main, radio services, plugin applications)
- `include/` - Headers (global.h for shared externs, html.h generated)
- `data/` - Web UI (index.html)
- `lib/` - (currently empty; external libs via platformio.ini)
- `test/` - (test fixtures, not CI-integrated)
