#include <Servo.h>

Servo servo_0;
Servo servo_1;
Servo servo_2;
Servo servo_3;

int sensorPin0 = A0; // Schulter
int sensorPin1 = A1; // Hand
int sensorPin2 = A2; // Ellbogen
int sensorPin3 = A3; // Zange

int arrayStep = 0;
int arrayMax = 0;
int stepsMax = 180;
int time = 1000;
int del = 1000;

// arrays
int Delay[7] = {0, 0, 1, 3, 15, 60, 300}; // array to map gripper pot to delay in seconds
int SensVal[4]; // sensor value
float dif[4], ist[4], sol[4], dir[4]; // difference between stored position and momentary position
int joint0[180]; // array for servo(s)
int joint1[180];
int joint2[180];
int joint3[180];

int top = 179; // we should not write over the end of an array
boolean playmode = true;
unsigned long previousMillis1 = 0;
unsigned long currentMillis = 0;

void setup() {
  pinMode(13, OUTPUT); // sets the digital pin 13 as output
  digitalWrite(13, HIGH); // sets the LED on
  servo_0.attach(9); // attaches the servo
  servo_1.attach(5);
  servo_2.attach(20);
  servo_3.attach(11);
  Serial.begin(115200); // Baudrate must be the same on the IDE
  Serial.println("mini robot ready...");
  digitalWrite(13, LOW);
  
  // Initialize joint arrays
  for (int i = 0; i < 180; i++) {
    joint0[i] = i;
    joint1[i] = i;
    joint2[i] = i;
    joint3[i] = i;
  }
  
  arrayMax = 180; // Set the maximum number of steps in the array
}

void loop() {
  currentMillis = millis(); // Update current time
  
  if (playmode && arrayStep < arrayMax) {
    if (currentMillis - previousMillis1 > time) {
      previousMillis1 = currentMillis;
      readPot(); // Read values from potentiometers
      mapping(); // Map values to servo positions
      move_servo(); // Move the servos to the new positions
      arrayStep++;
      if (arrayStep >= arrayMax) {
        arrayStep = 0; // Reset arrayStep to loop back to the beginning
      }
    }
  }
}

void readPot() {
  SensVal[0] = analogRead(sensorPin0);
  SensVal[1] = analogRead(sensorPin1);
  SensVal[2] = analogRead(sensorPin2);
  SensVal[3] = analogRead(sensorPin3);
}

void mapping() {
  ist[0] = SensVal[0];
  sol[0] = map(ist[0], 0, 1023, 0, 179);
  dir[0] = abs(joint0[arrayStep] - sol[0]) / stepsMax;
  
  ist[1] = SensVal[1];
  sol[1] = map(ist[1], 0, 1023, 0, 179);
  dir[1] = abs(joint1[arrayStep] - sol[1]) / stepsMax;
  
  ist[2] = SensVal[2];
  sol[2] = map(ist[2], 0, 1023, 0, 179);
  dir[2] = abs(joint2[arrayStep] - sol[2]) / stepsMax;
  
  ist[3] = SensVal[3];
  sol[3] = map(ist[3], 0, 1023, 0, 179);
  dir[3] = abs(joint3[arrayStep] - sol[3]) / stepsMax;
}

void move_servo() {
  if (joint0[arrayStep] > sol[0]) {
    servo_0.write(joint0[arrayStep] - dir[0]);
  } else if (joint0[arrayStep] < sol[0]) {
    servo_0.write(joint0[arrayStep] + dir[0]);
  }
  
  if (joint1[arrayStep] > sol[1]) {
    servo_1.write(joint1[arrayStep] - dir[1]);
  } else if (joint1[arrayStep] < sol[1]) {
    servo_1.write(joint1[arrayStep] + dir[1]);
  }
  
  if (joint2[arrayStep] > sol[2]) {
    servo_2.write(joint2[arrayStep] - dir[2]);
  } else if (joint2[arrayStep] < sol[2]) {
    servo_2.write(joint2[arrayStep] + dir[2]);
  }
  if (joint3[arrayStep] > sol[3]) {
    servo_3.write(joint3[arrayStep] - dir[3]);
  } else if (joint3[arrayStep] < sol[3]) {
    servo_3.write(joint3[arrayStep] + dir[3]);
  }
  
  delay(del);
}
