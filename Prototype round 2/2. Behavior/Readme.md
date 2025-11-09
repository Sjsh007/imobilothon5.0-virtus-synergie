Behavioral Monitor

Role:Driver fatigue and distraction monitor.

Platform:Python (likely Raspberry Pi, as it uses picamera2).

Logic:Uses *MediaPipe Face Mesh* to track facial landmarks in real-time from a camera feed.

Triggers:

    Fatigue (break.mp3):Detects if eyes are closed for more than 3 seconds (microsleep), if the driver yawns 3+ times in 60 seconds, or if the PERCLOS (Percentage of Eye Closure) score is too high.

    Distraction (focus.mp3):Detects if the driver's head is turned away from the road for more than 1.2 seconds.
    
Action:Plays local audio files (break.mp3 or focus.mp3) to alert the driver. It does not stop the vehicle or send an SMS.