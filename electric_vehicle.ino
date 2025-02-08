#include <LiquidCrystal_I2C.h>

// Motor Encoder Pins
#define ENCODER2_A 2  // Motor 1 Encoder A (Interrupt)
#define ENCODER2_B 4  // Motor 1 Encoder B
#define ENCODER1_A 3  // Motor 2 Encoder A (Interrupt)
#define ENCODER1_B 7  // Motor 2 Encoder B

// Motor Control Pins (DRV8833)
#define MOTOR1_A 5  // Motor 1 Input A
#define MOTOR1_B 6  // Motor 1 Input B
#define MOTOR2_A 9  // Motor 2 Input A
#define MOTOR2_B 10 // Motor 2 Input B

// buttons
#define INC_BTN 8 // button that increases speed
#define DEC_BTN 12 // button that decreases speed
#define START_BTN 11 // button that starts vehicle

LiquidCrystal_I2C lcd(0x27,16,2);

// Encoder Fixes
float kp = 1.2;
// float ki = 0.00005;
float ki = 0.0;
float kd = 0.1;
float prev_time = 0.0;
float total = 0.0;
float error = 0.0;
float prev_error = 0.0;
float derivative = 0.0;
volatile long encoder1_count = 0;
volatile long encoder2_count = 0;
volatile long max_encoder_count = 0;

//speeding up
int slow_speed = 100;
int fast_speed = 255;
int accelerating_ticks = 300;// ticks it takes to accelerate from slow to fast or vice versa

//speed
int speed1 = slow_speed;
int speed2 = slow_speed;

float distance = 3.0; //distance vehicle goes cm
float interval = 0.25;

// Interrupt Service Routines for Encoder Counting
void encoder1_ISR() {
  if (digitalRead(ENCODER1_B) == HIGH) encoder1_count++;
  else encoder1_count--;
}

void encoder2_ISR() {
  if (digitalRead(ENCODER2_B) == HIGH) encoder2_count--;
  else encoder2_count++;
}

void setup() {
  Serial.begin(9600);

  //buttons
  pinMode(INC_BTN, INPUT);
  pinMode(DEC_BTN, INPUT);
  pinMode(START_BTN, INPUT);

  //motor pins as output
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);

  //encoder pins as input
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  //attach tnterrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2_ISR, RISING);
  
  //lcd stuff
  lcd.init();
  lcd.clear();
  lcd.backlight();      //make sure backlight is on
  lcd.setCursor(2,0);   //set cursor to character 2 on line 0
  prev_time = millis();
}

long cmToTick(long cm){
  return cm*240*7*60/4/(3.14159*6.0) * 1/1.8;
}

void updateDistance() {
  distance += interval*digitalRead(INC_BTN) - interval*digitalRead(DEC_BTN);
  lcd.clear();        
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print(distance);

}

void loop() {

  while(digitalRead(START_BTN) == LOW) {
    updateDistance();
    delay(150);
  }
  delay(200);

  // reset encoder counts
  encoder1_count = 0;
  encoder2_count = 0;
  max_encoder_count = 0;

  total = 0.0;

  long ticks = cmToTick(distance);
  Serial.print("ticks: ");
  Serial.print(ticks);
  Serial.print(" distance: ");
  Serial.println(distance);

  while (max_encoder_count<ticks){
    Serial.print("Encoder1: ");
    Serial.print(encoder1_count);
    Serial.print(" | Encoder2: ");
    Serial.print(encoder2_count);

    max_encoder_count = max(encoder1_count, encoder2_count);
    if (max_encoder_count<accelerating_ticks){
      speed1 = (float)max_encoder_count/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
      speed2 = (float)max_encoder_count/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
    }
    if (ticks - max_encoder_count < accelerating_ticks){
      speed1 = (float)(ticks - max_encoder_count)/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
      speed2 = (float)(ticks - max_encoder_count)/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
    }
    error = encoder1_count - encoder2_count;
    float current = millis();
    total += error * (current - prev_time);
    derivative = (error - prev_error)/(current-prev_time);
    prev_error = error;
    prev_time = current;
    float adjustment = error * kp + total * ki + derivative * kd;
    Serial.print(" adjustment ");
    Serial.println(adjustment);
    analogWrite(MOTOR1_A, constrain(speed1-adjustment, 15, 255));
    digitalWrite(MOTOR1_B, LOW);

    analogWrite(MOTOR2_A, constrain(speed2+adjustment, 15, 255));
    digitalWrite(MOTOR2_B, LOW);
  }
  // turn off both motors
  analogWrite(MOTOR1_A, LOW);
  digitalWrite(MOTOR1_B, LOW);

  analogWrite(MOTOR2_A, LOW);
  digitalWrite(MOTOR2_B, LOW);

  // // Print encoder counts
  // Serial.print("Encoder1: ");
  // Serial.print(encoder1_count);
  // Serial.print(" | Encoder2: ");
  // Serial.println(encoder2_count);

  delay(200);
}
