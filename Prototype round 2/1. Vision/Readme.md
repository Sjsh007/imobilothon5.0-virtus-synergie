Steering Monitor & Response

Role: An autonomous "driver-down" detector based on steering input.

Platform:ESP32.

Logic: Actively monitors a potentiometer (simulating a steering wheel). It uses a state machine to track the driver's responsiveness.

Triggers:

    Warning (10s of no movement):* Sounds a buzzer.
    Emergency (15s of no movement):* Triggers a full emergency (STATE_EMERGENCY_RAMP).
    
Action:

    1.  Flashes an LED and sounds the buzzer.
    2.  Ramps down the motor speed over 10 seconds.
    3.  Applies an active brake to the motor.
    4.  Fetches the final GPS location.
    5.  Connects to WiFi and sends a Twilio SMS with the message "EMERGENCY: Driver Unresponsive (Steering)" and a map link.

    Cancel Feature:If the driver moves the wheel during the ramp-down, the emergency is *cancelled*, and the system returns to normal.
