# Entry Gate Access Control System

## Project Description
This project is an ESP32-based smart entry control system for an access gate or door. It uses RFID card verification, infrared safety sensors, motorized door control, WiFi connectivity, MQTT commands, and cloud-based permission sync to allow or deny access automatically.

The system is designed for a gate/door entry environment where authorized cards are stored locally on the ESP32 and periodically synced from a backend server. It supports both online cloud access and offline fallback behavior, making it suitable for smart hostel, office, or facility entry systems.

## Main Features
- RFID-based access control using RDM6300 readers
- Dual-direction access logic for entry and exit doors
- Local card list storage in LittleFS
- Cloud sync with ERP/api permission server
- MQTT command support for remote control and updates
- WiFi auto-connect with WiFiManager portal fallback
- Audio feedback using DFPlayer Mini
- IR sensor-based safety logic and door movement detection
- Emergency open/close commands and power-state handling
- Automatic retry and offline restart handling

## System Overview
The project runs on an ESP32 development board and controls:
- RFID reader inputs for inside and outside access points
- IR sensors to detect person movement and door states
- Motor driver outputs to drive left and right door motors
- Status LEDs and buzzer/audio signals
- WiFi and MQTT communication with the server
- HTTPS API sync to fetch authorized card numbers from the backend

## Hardware Used
This project is built around the following components:
- ESP32 microcontroller
- RDM6300 RFID readers (inside/outside)
- DFPlayer Mini MP3 module
- Speaker / buzzer output
- Door motor driver and motors
- IR obstacle / position sensors
- Status LEDs and signal pins
- 24V/AC power monitoring input

## Pin Configuration
The code defines the main pins for the system:
- RFID readers:
  - `P_RDM_INSIDE = 2`
  - `P_RDM_OUTSIDE = 1`
- Input sensors:
  - `P_IR_INSIDE = 35`
  - `P_IR_OUTSIDE = 36`
  - `P_L_DOOR_CLOSE_S = 37`
  - `P_R_DOOR_CLOSE_S = 38`
  - `P_L_DOOR_OPEN_S = 39`
  - `P_R_DOOR_OPEN_S = 40`
  - `AC_POWER = 21`
- Door signal outputs:
  - `INTER_SIGNAL = 41`
  - `EXIT_SIGNAL = 42`
- Motor outputs:
  - `PWM1_L = 4`
  - `PWM2_L = 5`
  - `PWM3_R = 6`
  - `PWM4_R = 7`

## Software Libraries Used
The Arduino sketch depends on the following libraries:
- `WiFi.h`
- `WiFiManager.h`
- `HTTPClient.h`
- `LittleFS.h`
- `ArduinoJson.h`
- `rdm6300.h`
- `PubSubClient.h`
- `DFRobotDFPlayerMini.h`
- `Preferences.h`

Install these libraries in the Arduino IDE or PlatformIO environment before uploading the sketch.

## Project Workflow
1. The ESP32 boots and initializes all pins, sensors, drivers, and storage.
2. It attempts to connect to WiFi using saved settings or the WiFiManager portal.
3. It starts MQTT communication and subscribes to device command topics.
4. It syncs card list from the server using HTTPS requests.
5. RFID readers scan cards for inside and outside access points.
6. If the scanned card is in the local authorized list, the door opens.
7. IR sensors verify the safe movement of the door and detect wrong-way entry.
8. Audio prompts indicate access grant, deny, power events, or step-back conditions.
9. If the server is unreachable, the device continues local operation using the last synced card list.

## Access Logic
The project supports two main modes:
- Entry access: authorized card from outside opens the door for entry
- Exit access: authorized card from inside opens the door for exit

When a card is read, the system compares it against:
- locally stored allowed card numbers in `/card.txt`
- a list of master cards defined in the code

If a card is not found, the device plays an access denied tone and schedules a sync retry.

## MQTT Commands
The device listens to MQTT topics in the format:
- `devices/<device_id>/cmd`

Supported command examples include:
- `{"cmd":"open"}`
- `{"cmd":"close"}`
- `{"cmd":"card","val":"1234567890"}`
- `{"cmd":"cards","val":["card1","card2"]}`
- `{"cmd":"speed","l_speed":180,"r_speed":180}`
- `{"cmd":"volume","volume":20}`
- `{"cmd":"get"}`
- `{"cmd":"reset"}`
- `{"cmd":"restart"}`
- `{"cmd":"format"}`

## HTTP Sync
The system periodically sends device data to access-control endpoints and fetches the latest permission list.

Main endpoints used by the code:
- `https://erp.neways3.com/api/access-device/information-access/update-device-status`
- `https://erp.neways3.com/api/access-device/information-access/get-permission`
- Fallback endpoints:
  - `https://cloud.neways3.com/api/access-device/information-access/update-device-status`
  - `https://cloud.neways3.com/api/access-device/information-access/get-permission`

The request payload includes:
- `api_secret`
- `device_code`

## Local Storage
The system stores the authorized card list in the ESP32 flash using LittleFS:
- `/card.txt`

This file stores one card number per line.

## Audio Messages
The device uses predefined audio track IDs for system states, including:
- `GET_READY_FOR_USE`
- `DATA_UPDATE`
- `DATA_UPDATE_FAILED`
- `WELCOME_TO_SUPER_HOME`
- `SEE_YOU_AGAIN`
- `POWER_CONNECTED`
- `POWER_DISCONNECTED`
- `EMERGENCY_OPENING`
- `EMERGENCY_CLOSING`
- `MACHINE_READY_FOR_USE`
- `WRONG_WAY`
- `STEP_BACK`
- `ACCESS_DENIED`

These audio macros are mapped to the DFPlayer MP3 numbers in the code.

## Important Notes
- `api_secret` is hardcoded in the project and should be replaced with your own secure value before deployment.
- MQTT username/password are included in the source and should be changed for production use.
- The sketch uses `WiFiManager` for setup and fallback portal mode.
- The code uses `setInsecure()` in HTTPS requests, which is acceptable for development/testing but should be upgraded to a properly validated certificate setup in production.
- This project is intended for embedded hardware control, so any deployment should be tested carefully on-site before live usage.

## Upload Instructions
1. Open the file `Entry-Get-Wifi-Code-Update-New-ERP.ino` in the Arduino IDE.
2. Install required libraries.
3. Select the correct ESP32 board and COM port.
4. Compile and upload the sketch.
5. Open the serial monitor to observe boot messages, WiFi connection, and device status.
6. If the ESP32 does not have saved WiFi settings, it will launch a WiFiManager configuration portal.

## File Structure
- `Entry-Get-Wifi-Code-Update-New-ERP.ino` – main embedded firmware
- `README.md` – project documentation

## Project Status
This firmware is a custom industrial/embedded access-control solution for real-world gate automation and automation systems.

## Suggested GitHub Repository Description
Smart ESP32 RFID gate access controller with WiFi, MQTT, cloud card sync, IR safety logic, and door motor control.

## Example Use Cases
- Hostel gate access
- Office entry control
- Smart door entry system
- Facility access automation
- Apartment and building security control

## Disclaimer
This project is provided for educational and engineering use. It is not a production-ready security system unless thoroughly tested and configured for your specific hardware and environment.
