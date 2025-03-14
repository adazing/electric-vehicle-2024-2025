
// Motor Encoder Pins
#define ENCODER_A 2  // Motor 1 Encoder A (Interrupt)
#define ENCODER_B 3  // Motor 1 Encoder B

// Motor Control Pins (DRV8833)
#define MOTOR_A 5  // Motor 1 Input A
#define MOTOR_B 6  // Motor 1 Input B
#define MOTOR_SPEED 9
// buttons
#define START_BTN 13 // button that starts vehicle

volatile long encoder_count = 0;

//speeding up
int slow_speed = 150;
int fast_speed = 200;
int accelerating_ticks = 300;// ticks it takes to accelerate from slow to fast or vice versa

//speed
int speed = slow_speed;

float distance = 3.0; //distance vehicle goes in meters
float cpr = 211.2;
int total_ticks = distance*cpr/(0.045*3.14159*2)/4*3/3.5;

// Interrupt Service Routines for Encoder Counting
void encoder_ISR() {
  if (digitalRead(ENCODER_B) == HIGH) encoder_count++;
  else encoder_count--;
}

void setup() {
  Serial.begin(9600);

  //buttons
  pinMode(START_BTN, INPUT);

  //motor pins as output
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(MOTOR_SPEED, OUTPUT);

  //encoder pins as input
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  //attach tnterrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_ISR, RISING);
  }


void loop() {
  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_B, LOW);
  while(digitalRead(START_BTN) == LOW) {
    delay(150);
  }
  delay(200);
  
  digitalWrite(MOTOR_A, HIGH);
  digitalWrite(MOTOR_B, LOW);

  // reset encoder counts
  encoder_count = 0;

  Serial.print("ticks: ");
  Serial.print(total_ticks);
  Serial.print(" distance: ");
  Serial.println(distance);

  while (encoder_count<total_ticks){
    Serial.print("encoder count: ");
    Serial.println(encoder_count);

    if (encoder_count<accelerating_ticks){
      speed = (float)encoder_count/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
    }
    if (total_ticks - encoder_count < accelerating_ticks){
      speed = (float)(total_ticks - encoder_count)/(float)accelerating_ticks*((float)fast_speed-(float)slow_speed)+(float)slow_speed;
    }
    analogWrite(MOTOR_SPEED, constrain(speed, 15, 255));

  }
  // turn off both motors
  digitalWrite(MOTOR_A, LOW);
  digitalWrite(MOTOR_B, LOW);


  delay(200);
}
