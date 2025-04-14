//some parts of this code are inspired by  https://github.com/upsidedownlabs/BioAmp-EXG-Pill  
#include <Servo.h>
#define NUM_SERVOS 5
#define SERVO_PIN_1 3//thumb
#define SERVO_PIN_2 5
#define SERVO_PIN_3 6
#define SERVO_PIN_4 9
#define SERVO_PIN_5 10//pinky

Servo servos[NUM_SERVOS];
#define WINDOW_DURATION 200 // Window duration in milliseconds
#define NUM_SAMPLES 100 
float emgBuffer[NUM_SAMPLES]; 
unsigned long startTime = 0;  // Time window start
unsigned long lastSampleTime = 0; // Time of the last sample
unsigned long sampleInterval;
int sampleCount = 0;   
float Q = 0.006;  // Process noise covariance
float R = 1;     // Measurement noise covariance
float x = 0;     // Initial state estimate
float P = 1;     // Initial estimate covariance
float K = 0;  
#define SAMPLE_RATE 500
#define BAUD_RATE 9600
#define INPUT_PIN A0
#define BUFFER_SIZE 128

#define EMG_MIN 2
#define EMG_MAX 10

int circular_buffer[BUFFER_SIZE];
int data_index, sum;
int ans[4]={150,2000,1050,500};
Servo servo;


// Envelop detection algorithm
int getEnvelop(int abs_emg){
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum/BUFFER_SIZE) * 2;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
void setup() {
  // Serial connection begin
  Serial.begin(BAUD_RATE);
  servos[0].attach(SERVO_PIN_1);
  servos[1].attach(SERVO_PIN_2);
  servos[2].attach(SERVO_PIN_3);
  servos[3].attach(SERVO_PIN_4);
  servos[4].attach(SERVO_PIN_5);
  // Attach servo
 
  sampleInterval = WINDOW_DURATION / NUM_SAMPLES; // Calculate interval (in ms)
  startTime = millis();         // Initialize the window start time
  lastSampleTime = millis();
}

void loop() {
  // Calculate elapsed time
    // Get current time
  unsigned long currentMillis = millis();
  // Add new sample to buffer at regular intervals
  while (sampleCount<NUM_SAMPLES) {
    // Read the EMG sensor value
    startTime=millis();
    static unsigned long past = 0;
    unsigned long present = micros();
    unsigned long interval = present - past;
    past = present;
  
    // Run timer
    static long timer = 0;
    timer -= interval;
  
    // Sample and get envelop
    if(timer < 0) {
      timer += 1000000 / SAMPLE_RATE;
      int sensor_value = analogRead(INPUT_PIN);
      float signal = EMGFilter(sensor_value);
      int envelop = getEnvelop(abs(signal));
      int servo_position = map(envelop, EMG_MIN, EMG_MAX, 0, 180);
      float x_pred = x;
      float P_pred = P + Q;
  
    // Update
      K = P_pred / (P_pred + R);
      x = x_pred + K * (servo_position - x_pred);
      P = (1 - K) * P_pred;
     // servo.write(servo_position);
      
      Serial.println(x);
//      Serial.print(",");
//      Serial.println(servo_position);
     
//      float emgValue=x;
//  
//      // Add the value to the buffer
//      if (sampleCount < NUM_SAMPLES) {
//        emgBuffer[sampleCount++] = emgValue;
//      }
//      if(millis()-startTime<20){
//          delay(millis()-startTime);
//        }
//  
//       // Update the last sample time
//    }
//  }
//  
//    // Once the 200 ms window is over, calculate the mean
//    
//      // Calculate the mean of the samples
//      float sum1 = 0;
//      for (int i = 0; i < sampleCount; i++) {
//        sum1 += emgBuffer[i];
//      }
//      float mean1 = sum1 / sampleCount;
//  
//      // Print the mean value
//      Serial.print("Mean EMG over 200 ms: ");
//      Serial.println(mean1);
      int close=2000,index1=0;
      int mean1=x;
      for(int t=0;t<4;t++){
          if(abs(ans[t]-mean1)<close){
              close=abs(ans[t]-mean1);
              index1=t;
            }
        }
       Serial.println(index1);
      switch(index1){
        case 3:
          servos[0].write(150); // Set to 90° (example position)
          servos[1].write(180);  // Reset others
          servos[2].write(180);
          servos[3].write(0);
          servos[4].write(150);
          Serial.println("victory");
          break;
        case 0:
          servos[0].write(0); // Set to 90° (example position)
          servos[1].write(180);  // Reset others
          servos[2].write(180);
          servos[3].write(180);
          servos[4].write(0);
          Serial.println("open");
          break;
        case 1:
          servos[0].write(150); // Set to 90° (example position)
          servos[1].write(0);  // Reset others
          servos[2].write(0);
          servos[3].write(0);
          servos[4].write(130);
          Serial.println("close");
          break;
        case 2:
          servos[0].write(0); // Set to 90° (example position)
          servos[1].write(180);  // Reset others
          servos[2].write(0);
          servos[3].write(0);
          servos[4].write(0);
          Serial.println("yoyo");
          break;
        default:
          servos[0].write(0); // Set to 90° (example position)
          servos[1].write(180);  // Reset others
          servos[2].write(180);
          servos[3].write(180);
          servos[4].write(0);
          break;
        }
  
      // Reset for the next window
     // memset(emgBuffer, 0, sizeof(emgBuffer)); 
      sampleCount = 0;
      startTime = millis();         // Reset the window start time
    
  
    // Optional: Small delay for stability
    //delay(1000);
    
    }
  }
}
