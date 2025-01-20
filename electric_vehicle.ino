#include <LiquidCrystal_I2C.h>

const int switchPin = 10;

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
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;
volatile int encoder1Direction = 1;  // 1 for forward, -1 for backward
volatile int encoder2Direction = 1;  // 1 for forward, -1 for backward

int speed = 200;

const int incDistButton = 12;
const int decDistButton = 13;
const double MULTIPLIER = 0.1; //in m

double distance;
double savedDistance = 7.0;

LiquidCrystal_I2C lcd(0x27,16,2);

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
  // Serial.println("hi");

  //lcd stuff
  lcd.init();
  // Serial.println("testing");
  lcd.clear();
  // Serial.println("blah!");
  lcd.backlight();      // Make sure backlight is on
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  // lcd.print("Hello world!");
  // Serial.println("Hello world!");


}

void loop() {
  while(digitalRead(switchPin) == LOW) {
    updateDistance();
    delay(100);
  }
  encoder1Position = 0;
  encoder2Position = 0;

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
  
  while (encoder1Position<ticks){
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
    Serial.print(encoder1Position);
    Serial.print(" Direction: ");
    Serial.println(encoder1Direction == 1 ? "Forward" : "Backward");

  }
    Serial.println("hii");
    analogWrite(EN1, 0);
    digitalWrite(IN1A, LOW);
    digitalWrite(IN1B, LOW);
    analogWrite(EN2, 0);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, LOW);
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
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);

    digitalWrite(IN1A, LOW);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN2B, LOW);
  } else {
    // Set speed using PWM
    analogWrite(EN1, speed);
    analogWrite(EN2, speed);

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
    encoder2Position--;
    encoder2Direction = 1;  // Forward
  } else {
    encoder2Position++;
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
