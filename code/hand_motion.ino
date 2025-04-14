//some parts of this code are inspired by  https://github.com/upsidedownlabs/BioAmp-EXG-Pill  
#if defined(ESP32) 
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif

#include <Wire.h>

// === SERVO PINS ===
#define NUM_FINGER_SERVOS 5
#define SERVO_PIN_1 3   // thumb
#define SERVO_PIN_2 5
#define SERVO_PIN_3 6
#define SERVO_PIN_4 9
#define SERVO_PIN_5 10  // pinky
#define WRIST_SERVO_PIN 11
const int numDiscreteAngles = 3;
int discreteAngles[numDiscreteAngles] = {45,135,180};

Servo fingerServos[NUM_FINGER_SERVOS];
Servo wristServo;

// === EMG PARAMETERS ===
#define WINDOW_DURATION 200
#define NUM_SAMPLES 100
#define SAMPLE_RATE 500
#define BAUD_RATE 9600
#define INPUT_PIN A0
#define BUFFER_SIZE 128
#define EMG_MIN 2
#define EMG_MAX 10

float emgBuffer[NUM_SAMPLES]; 
unsigned long sampleInterval;
int circular_buffer[BUFFER_SIZE];
int data_index, sum;
int sampleCount = 0;

float Q = 0.006, R = 1, x = 0, P = 1, K = 0;
int gestureThresholds[4] = {150, 2000, 1050, 500}; // open, close, yoyo, victory

// === MPU6050 PARAMETERS ===
const int MPU_ADDR = 0x68;
int16_t accZ;
float angle;
const int sampleCountMPU = 10;
float angleBuffer[sampleCountMPU];
int bufferIndex = 0;
unsigned long lastSampleTimeMPU = 0;

// === EMG FUNCTIONS ===
int getEnvelop(int abs_emg) {
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum / BUFFER_SIZE) * 2;
}

float EMGFilter(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - 0.05159732 * z1 - 0.36347401 * z2;
    output = 0.01856301 * x + 0.03712602 * z1 + 0.01856301 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -0.53945795 * z1 - 0.39764934 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - 0.47319594 * z1 - 0.70744137 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.00211112 * z1 - 0.74520226 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// === MPU FUNCTIONS ===
int16_t readAccZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3F); // ACCEL_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return Wire.read() << 8 | Wire.read();
}

// === SETUP ===
void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  for (int i = 0; i < NUM_FINGER_SERVOS; i++) {
    fingerServos[i].attach(SERVO_PIN_1 + 2 * i); // spacing by pin number
  }

  wristServo.attach(WRIST_SERVO_PIN);
  sampleInterval = WINDOW_DURATION / NUM_SAMPLES;
}

// === MAIN LOOP ===
void loop() {
  handleMPU();
  handleEMG();
}

// === HANDLE MPU SERVO ===
void handleMPU() {
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTimeMPU >= 100) {
    lastSampleTimeMPU = currentTime;

    accZ = readAccZ();
    angle = (float)(accZ - 14000) * 180.0 / (-18000 - 14000);
    angle = constrain(angle, 0, 180);

    angleBuffer[bufferIndex++] = angle;

    if (bufferIndex >= sampleCountMPU) {
      float sum = 0;
      for (int i = 0; i < sampleCountMPU; i++) sum += angleBuffer[i];
      float meanAngle = sum / sampleCountMPU;

      // === Snap to closest predefined angle ===
      int closestAngle = discreteAngles[0];
      int minDiff = abs(meanAngle - discreteAngles[0]);
      for (int i = 1; i < numDiscreteAngles; i++) {
        int diff = abs(meanAngle - discreteAngles[i]);
        if (diff < minDiff) {
          minDiff = diff;
          closestAngle = discreteAngles[i];
        }
      }
      Serial.println(meanAngle);
     
      wristServo.write(closestAngle);
      bufferIndex = 0;
    }
  }
}


// === HANDLE EMG TO FINGER CONTROL ===
void handleEMG() {
  static unsigned long pastMicros = 0;
  unsigned long nowMicros = micros();
  unsigned long interval = nowMicros - pastMicros;
  pastMicros = nowMicros;

  static long timer = 0;
  timer -= interval;

  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    int sensor_value = analogRead(INPUT_PIN);
    float signal = EMGFilter(sensor_value);
    int envelop = getEnvelop(abs(signal));
    int servo_position = map(envelop, EMG_MIN, EMG_MAX, 0, 180);

    float x_pred = x;
    float P_pred = P + Q;

    K = P_pred / (P_pred + R);
    x = x_pred + K * (servo_position - x_pred);
    P = (1 - K) * P_pred;

    int meanEMG = x;
    int closest = 2000, gestureIndex = 0;
    for (int t = 0; t < 4; t++) {
      int diff = abs(gestureThresholds[t] - meanEMG);
      if (diff < closest) {
        closest = diff;
        gestureIndex = t;
      }
    }

    switch (gestureIndex) {
      case 0: // open
        fingerServos[0].write(0);
        fingerServos[1].write(180);
        fingerServos[2].write(180);
        fingerServos[3].write(180);
        fingerServos[4].write(0);
        break;
      case 1: // close
        fingerServos[0].write(150);
        fingerServos[1].write(0);
        fingerServos[2].write(0);
        fingerServos[3].write(0);
        fingerServos[4].write(130);
        break;
      case 2: // yoyo
        fingerServos[0].write(0);
        fingerServos[1].write(180);
        fingerServos[2].write(0);
        fingerServos[3].write(0);
        fingerServos[4].write(0);
        break;
      case 3: // victory
        fingerServos[0].write(150);
        fingerServos[1].write(180);
        fingerServos[2].write(180);
        fingerServos[3].write(0);
        fingerServos[4].write(150);
        break;
    }

    Serial.print("Gesture Index: ");
    Serial.println(gestureIndex);
  }
}
