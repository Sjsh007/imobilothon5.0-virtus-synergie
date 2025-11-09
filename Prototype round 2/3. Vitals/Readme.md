Vitals Simulator GUI

Role: A PC-based GUI to simulate a driver's vital signs and send commands to the vitals.ino ESP32.

Platform:Python (uses tkinter for the GUI and pyserial for communication).

Logic:Creates a "smartwatch" interface with sliders for Heart Rate (HR) and Blood Oxygen (SpO2).

Triggers:
    When the user moves the sliders into a pre-defined "danger" zone (e.g., SpO2 < 88 or HR < 40).

Action:
    Sends a formatted string over the USB serial port:
    Emergency:E,HR,SPO2\n (e.g., E,145,87\n)
    Normal:N\n


Vitals-Based Response

Role: The ESP32 "receiver" that listens for commands from vitals.py.
Platform: ESP32.
Logic: Listens for incoming data on the Serial (USB) port.
Trigger: 
Receiving the E,... emergency string from vitals.py.

Action:
    1.  Flashes an LED.
    2.  Ramps down the motor speed and applies the active brake.
    3.  Fetches the final GPS location.
    4.  Connects to WiFi and sends a *Twilio SMS* with the message "EMERGENCY: Driver Unresponsive" and includes the specific *HR and SpO2 data* it received.
    
    Key Feature: Includes a syncClockToGPS() function to fix the ESP32's system time, which is crucial for establishing a secure (SSL/HTTPS) connection to the Twilio API.