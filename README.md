🤖 EMG + MPU Controlled Prosthetic Arm
    This project demonstrates a functional prosthetic hand using EMG (muscle signals) and MPU6050 (IMU) for motion control of fingers and wrist via servos.

🧠 Core Idea
    Muscle signals (EMG) control individual finger gestures.

    Wrist rotation is controlled via MPU6050’s Z-axis acceleration.
    
    Processed sensor data drives 6 servo motors (5 fingers + 1 wrist).

📦 Features
    Advanced Filtering: Multiple IIR filters + Kalman filter for clean EMG input.
    
    Gesture Recognition: EMG signal mapped to 4 hand gestures:
    
    Open ✋
    
    Close ✊
    
    Yoyo 🤙
    
    Victory ✌️

MPU Smoothing: 10-sample moving average + angle snapping to predefined positions.

📸 Media
    Finger Gestures	Wrist Rotation
    Add your own .gif or .mp4 files in the images/ folder and update names accordingly.

🛠️ Setup
    Board: Arduino UNO / Nano / ESP32
    
    Libraries:
    
    Servo.h / ESP32Servo.h
    
    Wire.h
    
    Connections:
    
    EMG → A0
    
    MPU6050 → I2C
    
    6 Servos → PWM pins (3, 5, 6, 9, 10, 11)

▶️ Run
    Upload the .ino sketch to your board. Open serial monitor at 9600 baud for debug logs. Attach electrodes properly for best EMG signal.

👏 Credits
    Some filtering logic and gesture inspiration from [UpsideDownLabs BioAmp-EXG-Pill](https://github.com/upsidedownlabs/BioAmp-EXG-Pill)
