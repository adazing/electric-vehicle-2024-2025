#include <LiquidCrystal_I2C.h>

const int switchPin = 4;

// Motor driver pins
const int ENA = 9;  // Motor speed control
const int IN1 = 7;  // Motor direction control
const int IN2 = 8;  // Motor direction control

// Encoder pins
const int encoderPinA = 2;  // Encoder Channel A to digital pin 2
const int encoderPinB = 3;  // Encoder Channel B to digital pin 3

// Variables to track encoder position and direction
volatile long encoderPosition = 0;
volatile int encoderDirection = 1;  // 1 for forward, -1 for backward
int speed = 200;

double distance;

LiquidCrystal_I2C lcd(0x27,16,2);

void setup() {
  // Set motor pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Set encoder pins as input
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Attach interrupt to encoder channel A
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, RISING);
  
  // Initialize serial for debugging
  Serial.begin(9600);

  //lcd stuff
  lcd.init();
  lcd.clear();
  lcd.backlight();      // Make sure backlight is on
  // lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  // lcd.print("Hello world!");


}

void loop() {
  while(digitalRead(switchPin) == LOW) {
    updateDistance();
    delay(100);
  }
  encoderPosition = 0;
  // Example: Set motor speed and direction
  setMotorSpeed(150);  // Set speed (0 to 255)
  setMotorDirection(true);  // true = forward, false = backward
  // int ticks = cmToTick(distance);
  distance*=6.94/3.94;
  long ticks = cmToTick(distance);
  long speeding_up_distance = cmToTick(10);
  Serial.print("ticks: ");
  Serial.print(ticks);
  Serial.print(" distance: ");
  Serial.println(distance);
  
  while (encoderPosition<ticks){
    if (ticks<speeding_up_distance){
      Serial.print("encoderPosition");
      Serial.println(ticks);
      setMotorSpeed(150);
      // setMotorSpeed(speed*encoderPosition/speeding_up_distance);
    }else{
      setMotorSpeed(speed);
    }
    delay(100);
    Serial.print("Goal: ");
    Serial.print(ticks);
    Serial.print(" Position: ");
    Serial.print(encoderPosition);
    Serial.print(" Direction: ");
    Serial.println(encoderDirection == 1 ? "Forward" : "Backward");

  }
    Serial.println("hii");
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    delay(2000);
    // while(true){
    //     delay(100);
    // }

  // setMotorSpeed(0);

  // Print encoder position and direction for debugging
  // Serial.print("Position: ");
  // Serial.print(encoderPosition);
  // Serial.print(" Direction: ");
  // Serial.println(encoderDirection == 1 ? "Forward" : "Backward");
  
  // delay(100);  // Delay for readability
}

long cmToTick(long cm){
  return cm*240*7/4/(3.14159*6.0);
}

// Function to set motor speed (0-255)
void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);  // Limit speed to 0-255
  if (speed == 0) {
    // Apply brake to stop the motor immediately
    // analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  } else {
    // Set speed using PWM
    analogWrite(ENA, speed);
  }
}


// Function to set motor direction
void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

// Interrupt service routine for encoder
void readEncoder() {
  int stateB = digitalRead(encoderPinB);
  
  // Determine direction based on channel B relative to channel A
  if (stateB == digitalRead(encoderPinA)) {
    encoderPosition--;
    encoderDirection = 1;  // Forward
  } else {
    encoderPosition++;
    encoderDirection = -1;  // Backward
  }
}

void updateDistance() {
  float voltage = analogRead(A0) / 1024.0 * 5.0;
  distance = ((voltage - 0.1) * 3.0 / 4.9) + 7;
  lcd.clear();        
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print(distance);
  distance *= 100;
}
