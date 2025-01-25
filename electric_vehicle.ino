#include <LiquidCrystal_I2C.h>

const int switchPin = 11;

// Motor driver pins
const int EN1 = 9;  // Motor speed control
const int IN1A = 7;  // Motor direction control
const int IN1B = 8;  // Motor direction control

const int EN2 = 6;  // Motor speed control
const int IN2A = 5;  // Motor direction control
const int IN2B = 4;  // Motor direction control

// Encoder pins
const int encoder1PinA = 2;  // Encoder Channel A to digital pin 2
const int encoder2PinA = 3;  // Encoder Channel A to digital pin 3
const int encoder1PinB = A1;
const int encoder2PinB = A2;


// Variables to track encoder position and direction
volatile long encoderPosition = 0;
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;
volatile int encoder1Direction = 1;  // 1 for forward, -1 for backward
volatile int encoder2Direction = 1;  // 1 for forward, -1 for backward

// int speed1 = 160;
// int speed2 = 240;
int slow_speed = 100;
int fast_speed = 255;
int accelerating_ticks = 500;// ticks it takes to accelerate from slow to fast or vice versa
bool accelerated = false;
int speed1 = slow_speed;
int speed2 = slow_speed;

const int incDistButton = 12;
const int decDistButton = 13;
const double MULTIPLIER = 0.01; //in m

double distance;
double savedDistance = 7.0;

LiquidCrystal_I2C lcd(0x27,16,2);

float kp = 0.7;
float ki = 0.00004;
float kd = 0.12;
float prev_time = 0.0;
float total = 0.0;
float error = 0.0;
float prev_error = 0.0;
float derivative = 0.0;

void setup() {
  // Set motor pins as output
  pinMode(EN1, OUTPUT);
  pinMode(IN1A, OUTPUT);
  pinMode(IN1B, OUTPUT);

  pinMode(EN2, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(IN2B, OUTPUT);

  // Set encoder pins as input
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  // Attach interrupt to encoder channel A
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), readEncoderMotor1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), readEncoderMotor2, RISING);

  
  // Initialize serial for debugging
  Serial.begin(9600);

  //lcd stuff
  lcd.init();
  lcd.clear();
  lcd.backlight();      // Make sure backlight is on
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  prev_time = millis();

}

void loop() {
    analogWrite(EN1, 0);
    digitalWrite(IN1A, LOW);
    digitalWrite(IN1B, LOW);
    analogWrite(EN2, 0);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, LOW);
  while(digitalRead(switchPin) == LOW) {
    updateDistance();
    delay(50);
  }
  
  delay(100);
  encoder1Position = 0;
  encoder2Position = 0;

  total = 0.0;
  // accelerated = false;

  // Example: Set motor speed and direction
  setMotorDirection(true);  // true = forward, false = backward
  distance *= 7/7.66;
  // distance*=6.94/3.94;
  long ticks = cmToTick(distance);
  Serial.print("ticks: ");
  Serial.print(ticks);
  Serial.print(" distance: ");
  Serial.println(distance);
  while (encoderPosition<ticks){
    encoderPosition = max(encoder1Position, encoder2Position);
    Serial.print(" encoderPosition ");
    Serial.print(encoderPosition);
    Serial.print(" blah ");
    Serial.print(encoderPosition/accelerating_ticks);
    if (encoderPosition<accelerating_ticks){
      // int change = (fast_speed-slow_speed)/accelerating_ticks;
      // int change = fast_speed - slow_speed;
      speed1 = (float)encoderPosition/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
      speed2 = (float)encoderPosition/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
      // accelerated = true;
      // total = 0.0; //reset integral
    }
    // delay(5);
    // Serial.print("dt");
    Serial.print(" button ");
    Serial.print(digitalRead(switchPin));
    Serial.print(" encoder 1 position: ");
    Serial.print(encoder1Position);
    Serial.print(" encoder 2 position:");
    Serial.print(encoder2Position);
    Serial.print(" max ticks: ");
    Serial.print(ticks);
    error = encoder1Position - encoder2Position;
    float current = millis();
    total += error * (current - prev_time);
    derivative = (error - prev_error)/(current-prev_time);
    prev_error = error;
    prev_time = current;
    float adjustment = error * kp + total * ki + derivative * kd;
    // if(!accelerated){
    //   adjustment = false;
    // }
    Serial.print(" adjustment ");
    Serial.print(adjustment);
    Serial.print(" speed1 ");
    Serial.print(constrain(speed1 - adjustment, 0, 255));
    Serial.print(" speed2 ");
    Serial.println(constrain(speed2 + adjustment, 0, 255));
    setMotorSpeed1(speed1 - adjustment);
    setMotorSpeed2(speed2 + adjustment);

  }
    encoder1Position = 0;
    encoder2Position = 0;
    encoderPosition = 0;
    Serial.println("hii");
    analogWrite(EN1, 0);
    digitalWrite(IN1A, LOW);
    digitalWrite(IN1B, LOW);
    analogWrite(EN2, 0);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, LOW);
    delay(2000);
}

long cmToTick(long cm){
  return cm*240*7/4/(3.14159*6.0);
}

// Function to set motor speed (0-255)
void setMotorSpeed1(int speed) {
  speed = constrain(speed, 0, 255);  // Limit speed to 0-255
  if (speed == 0) {
    // Apply brake to stop the motor immediately
    analogWrite(EN1, 0);

    digitalWrite(IN1A, LOW);
    digitalWrite(IN1B, LOW);
  } else {
    analogWrite(EN1, speed);

    digitalWrite(IN1A, HIGH);
    digitalWrite(IN1B, LOW);

  }
}

// Function to set motor speed (0-255)
void setMotorSpeed2(int speed) {
  speed = constrain(speed, 0, 255);  // Limit speed to 0-255
  if (speed == 0) {
    // Apply brake to stop the motor immediately
    analogWrite(EN2, 0);

    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, LOW);
  } else {
    analogWrite(EN2, speed);

    digitalWrite(IN2A, HIGH);
    digitalWrite(IN2B, LOW);
    // Set speed using PWM

  }
}

// Function to set motor direction
void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(IN1A, HIGH);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2A, HIGH);
    digitalWrite(IN2B, LOW);
  } else {
    digitalWrite(IN1A, LOW);
    digitalWrite(IN1B, HIGH);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, HIGH);
  }
}

// Interrupt service routine for encoder
void readEncoderMotor1() {
  int stateB = digitalRead(encoder1PinB);
  
  // Determine direction based on channel B relative to channel A
  if (stateB == digitalRead(encoder1PinA)) {
    encoder1Position--;
    encoder1Direction = 1;  // Forward
  } else {
    encoder1Position++;
    encoder1Direction = -1;  // Backward
  }
}

// Interrupt service routine for encoder
void readEncoderMotor2() {
  int stateB = digitalRead(encoder2PinB);
  
  // Determine direction based on channel B relative to channel A
  if (stateB == digitalRead(encoder2PinA)) {
    encoder2Position++;
    encoder2Direction = 1;  // Forward
  } else {
    encoder2Position--;
    encoder2Direction = -1;  // Backward
  }
}

void updateDistance() {
  savedDistance += MULTIPLIER*digitalRead(incDistButton) - MULTIPLIER*digitalRead(decDistButton);
  lcd.clear();        
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print(savedDistance);
  distance = savedDistance * 100.0;

}
