#include <Adafruit_NeoPixel.h>

// Define pin assignments
#define NEOPIXEL_PIN 10
#define POTENTIOMETER_PIN A2
#define LINE_SENSOR_OUT_PIN A0
#define S0_PIN 7
#define S1_PIN 8
#define S2_PIN 12
#define S3_PIN 13
#define TRIG_PIN 4
#define ECHO_PIN 2
#define MPIN00 5  // Left motor forward
#define MPIN01 6  // Left motor backward
#define MPIN10 3  // Right motor forward
#define MPIN11 11 // Right motor backward

// Motor speed range
#define MIN_SPEED 0
#define MAX_SPEED 255

// Line sensor properties
#define NUM_SENSORS 16

// NeoPixel setup
#define NUM_LEDS 16
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Variables
int sensorValues[NUM_SENSORS];

// Function to set up multiplexer for line sensor
void selectSensor(int index) {
  digitalWrite(S0_PIN, index & 0x01);
  digitalWrite(S1_PIN, (index >> 1) & 0x01);
  digitalWrite(S2_PIN, (index >> 2) & 0x01);
  digitalWrite(S3_PIN, (index >> 3) & 0x01);
}

// Read all line sensors
void readLineSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectSensor(i);
    delayMicroseconds(50); // Allow time for switching
    sensorValues[i] = digitalRead(LINE_SENSOR_OUT_PIN);
  }
}

// Function to control motors using direct digital control 
void controlMotors(int leftMotorSpeed, int rightMotorSpeed) {
  // Right motor control
  if (rightMotorSpeed > 0) {
    digitalWrite(MPIN10, HIGH);  // Move forward
    digitalWrite(MPIN11, LOW);   // Set backward to LOW
    analogWrite(MPIN10, rightMotorSpeed);  // PWM speed
  } else if (rightMotorSpeed < 0) {
    digitalWrite(MPIN10, LOW);   // Set forward to LOW
    digitalWrite(MPIN11, HIGH);  // Move backward
    analogWrite(MPIN11, -rightMotorSpeed); // PWM speed (positive)
  } else {
    digitalWrite(MPIN10, LOW);   // Stop motor
    digitalWrite(MPIN11, LOW);   // Stop motor
  }

  // Left motor control
  if (leftMotorSpeed > 0) {
    digitalWrite(MPIN00, HIGH);  // Move forward
    digitalWrite(MPIN01, LOW);   // Set backward to LOW
    analogWrite(MPIN00, leftMotorSpeed);  // PWM speed
  } else if (leftMotorSpeed < 0) {
    digitalWrite(MPIN00, LOW);   // Set forward to LOW
    digitalWrite(MPIN01, HIGH);  // Move backward
    analogWrite(MPIN01, -leftMotorSpeed); // PWM speed (positive)
  } else {
    digitalWrite(MPIN00, LOW);   // Stop motor
    digitalWrite(MPIN01, LOW);   // Stop motor
  }
}

// Set LED colors based on direction
void setLEDs(String direction) {
  if (direction == "left") {
    strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS);
  } else if (direction == "right") {
    strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS);
  } else if (direction == "forward") {
    strip.fill(strip.Color(0, 0, 255), 0, NUM_LEDS);
  } else {
    strip.fill(strip.Color(0, 0, 0), 0, NUM_LEDS);
  }
  strip.show();
}

// Measure distance with ultrasonic sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

void setup() {
  // Initialize pins (unchanged)
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(MPIN00, OUTPUT);
  pinMode(MPIN01, OUTPUT);
  pinMode(MPIN10, OUTPUT);
  pinMode(MPIN11, OUTPUT);

  // Initialize NeoPixel
  strip.begin();
  strip.show();

  Serial.begin(9600);

  // Set PWM frequency for motors 
  TCCR0B = TCCR0B & B11111000 | B00000010; // Set frequency to ~7.8 kHz
  TCCR2B = TCCR2B & B11111000 | B00000010;  // Set prescaler to 8 (for pin 3)
  TCCR1B = TCCR1B & B11111000 | B00000010;  // Set prescaler to 8 (for pin 11)

}

void turn180(int motorSpeed) {
  // Calculate the turn time based on the motor speed
  // The time should be inversely proportional to the motor speed
  // The faster the speed, the shorter the time required to turn 180 degrees
  //int turnTime = map(motorSpeed, MIN_SPEED, MAX_SPEED, BASE_TURN_TIME * 2, BASE_TURN_TIME);

  // Rotate the robot 180 degrees (left motor goes backward, right motor goes forward)
  controlMotors(motorSpeed, motorSpeed);
  Serial.println("Turning...");
  // Wait for the robot to complete the 180-degree turn
  delay(13000);

  // Stop the motors after the turn
  controlMotors(0, 0);
  Serial.println("Finished turning");
  delay(13000);
}


void loop() {
  // Read potentiometer for motor speed
  int potValue = analogRead(POTENTIOMETER_PIN);
  int motorSpeed = map(potValue, 0, 1023, MIN_SPEED, MAX_SPEED);

  // Read line sensors
  readLineSensors();

  // Determine direction based on sensor values
  int leftSum = 0, rightSum = 0;
  
  leftSum = sensorValues[2];
  rightSum = sensorValues[13];
  Serial.print("Left sum is: ");
    Serial.print(leftSum);
    Serial.print(", Right sum is: ");
    Serial.println(rightSum);

  String direction;

  if (leftSum == HIGH && rightSum == HIGH)
  {
    direction = "forward";
    controlMotors(motorSpeed, -motorSpeed);
  }
  //If right sensor detects black line, then turn right
  else if (rightSum == LOW && leftSum == HIGH )
  {
      direction = "left";
      controlMotors(0, -motorSpeed); // Turn left
  }
  //If left sensor detects black line, then turn left  
  else if (rightSum == HIGH && leftSum == LOW)
  {
      direction = "right";
      controlMotors(motorSpeed, 0); // Turn right
  } 
  //If both the sensors detect black line, then stop 
  else 
  {
    controlMotors(0, 0);
  }

  // Update LEDs based on direction
  setLEDs(direction);

  // Check obstacle distance
  long distance = measureDistance();
  if (distance < 10) { // Stop if obstacle is close
    controlMotors(0, 0);
    setLEDs("stop");
    turn180(motorSpeed);
  }

  delay(100);
}