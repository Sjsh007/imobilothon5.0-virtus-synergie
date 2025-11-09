AAI-Enhanced Driver Wellness Monitoring

    continuously fetch the vehicle's location.
Twilio Integration: If it receives the "E" (Emergency) command, it connects to WiFi, formats an alert message with the vitals and GPS data, and sends it as an SMS via the Twilio API.
Vehicle Control: Includes logic to control a motor (via an L298N driver), simulating a gradual vehicle slowdown during an emergency ramp-down.


Prerequisites:
Python (Vision & Vitals):
Python 3.x
opencv-python
mediapipe
pyserial
requests
Pillow (PIL)
picamera2 (if running on a Raspberry Pi)

Bash
pip install opencv-python mediapipe pyserial requests pillow picamera2
Arduino (ESP32):
ESP32 Board
Arduino IDE or PlatformIO

Libraries:
WiFi.h
WiFiClientSecure.h
base64.h
TinyGPSPlus.h
HardwareSerial.h

Configuration
final_esp2_code:
Update ssid and password with your WiFi credentials.
Update Twilio API credentials: accountSID, authToken, fromNumber, and toNumber.

Python Scripts:
Ensure the serial port (e.g., /dev/ttyUSB0 or COM3) is correct in both main code(vision)...py and final_python_code
