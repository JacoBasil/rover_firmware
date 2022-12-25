#define USE_USBCON 

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>

//motor PINS: A - PWM, B - direction
/*
#define MOTOR_1_PWM 3
#define MOTOR_1_DIR 2
#define MOTOR_1_SLP 41

#define MOTOR_2_PWM 5
#define MOTOR_2_DIR 4
#define MOTOR_2_SLP 35

#define MOTOR_3_PWM 7
#define MOTOR_3_DIR 6
#define MOTOR_3_SLP 45

#define MOTOR_4_PWM 10
#define MOTOR_4_DIR 9
#define MOTOR_4_SLP 51
*/
#define MOTOR_1_PWM 9
#define MOTOR_1_DIR 27
#define MOTOR_1_SLP 23

#define MOTOR_2_PWM 8
#define MOTOR_2_DIR 28
#define MOTOR_2_SLP 25

#define MOTOR_3_PWM 7
#define MOTOR_3_DIR 26
#define MOTOR_3_SLP 31

#define MOTOR_4_PWM 6
#define MOTOR_4_DIR 29
#define MOTOR_4_SLP 33

//current sensing PINS
#define MOTOR_1_CS A8
#define MOTOR_2_CS A9
#define MOTOR_3_CS A10
#define MOTOR_4_CS A11

//encoder PINS
#define ENCODER_1_A 30
#define ENCODER_1_B 31
#define ENCODER_2_A 22
#define ENCODER_2_B 23
#define ENCODER_3_A 12
#define ENCODER_3_B 13
#define ENCODER_4_A 26
#define ENCODER_4_B 27

//encoder ticks counter
volatile long enc_tick_1 = 0;
volatile long enc_tick_2 = 0;
volatile long enc_tick_3 = 0;
volatile long enc_tick_4 = 0;

//encoder signals state
volatile bool sig_enc_1_A = false;
volatile bool sig_enc_1_B = false;
volatile bool sig_enc_2_A = false;
volatile bool sig_enc_2_B = false;
volatile bool sig_enc_3_A = false;
volatile bool sig_enc_3_B = false;
volatile bool sig_enc_4_A = false;
volatile bool sig_enc_4_B = false;

enum MOTORS {MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4};

// 10ms interval for measurements
const int interval = 10;
long previousMillis = 0;
long currentMillis = 0;


ros::NodeHandle nh;

// Keep track of the number of wheel ticks
 
std_msgs::Int32 enc_4_tick_count;
ros::Publisher motor_4_Pub("enc_front_left_ticks", &enc_4_tick_count);

//Interrupt Service Routine for encoder #1
void encoderReadA1() {
  sig_enc_1_A = digitalRead(ENCODER_1_A);
  if(sig_enc_1_B != sig_enc_1_A) 
    enc_tick_1++;
  else enc_tick_1--;
}

void encoderReadB1() {
  sig_enc_1_B = digitalRead(ENCODER_1_B);
  if(sig_enc_1_B == sig_enc_1_A) enc_tick_1++;
  else enc_tick_1--;
}

//Interrupt Service Routine for encoder #2
void encoderReadA2() {
  sig_enc_2_A = digitalRead(ENCODER_2_A);
  if(sig_enc_2_B != sig_enc_2_A) enc_tick_2++;
  else enc_tick_2--;
}

void encoderReadB2() {
  sig_enc_2_B = digitalRead(ENCODER_2_B);
  if(sig_enc_2_B == sig_enc_2_A) enc_tick_2++;
  else enc_tick_2--;
}

//Interrupt Service Routine for encoder #3
void encoderReadA3() {
  sig_enc_3_A = digitalRead(ENCODER_3_A);
  if(sig_enc_3_B != sig_enc_3_A) enc_tick_3++;
  else enc_tick_3--;
}

void encoderReadB3() {
  sig_enc_3_B = digitalRead(ENCODER_3_B);
  if(sig_enc_3_B == sig_enc_3_A) enc_tick_3++;
  else enc_tick_3--;
}

//Interrupt Service Routine for encoder #4
void encoderReadA4() {
  sig_enc_4_A = digitalRead(ENCODER_4_A);
  if(sig_enc_4_B != sig_enc_4_A) enc_tick_4++;
  else enc_tick_4--;
}

void encoderReadB4() {
  sig_enc_4_B = digitalRead(ENCODER_4_B);
  if(sig_enc_4_B == sig_enc_4_A) enc_tick_4++;
  else enc_tick_4--;
}

void setup() {
  
  //Serial.begin(115200);
  //while (!Serial);
  
  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_1_DIR, OUTPUT);
  pinMode(MOTOR_1_SLP, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);
  pinMode(MOTOR_2_DIR, OUTPUT);
  pinMode(MOTOR_2_SLP, OUTPUT);
  pinMode(MOTOR_3_PWM, OUTPUT);
  pinMode(MOTOR_3_DIR, OUTPUT);
  pinMode(MOTOR_3_SLP, OUTPUT);
  pinMode(MOTOR_4_PWM, OUTPUT);
  pinMode(MOTOR_4_DIR, OUTPUT);
  pinMode(MOTOR_4_SLP, OUTPUT);
  pinMode(ENCODER_1_A, INPUT);
  pinMode(ENCODER_1_B, INPUT);
  pinMode(ENCODER_2_A, INPUT);
  pinMode(ENCODER_2_B, INPUT);
  pinMode(ENCODER_3_A, INPUT);
  pinMode(ENCODER_3_B, INPUT);
  pinMode(ENCODER_4_A, INPUT);
  pinMode(ENCODER_4_B, INPUT);
  sig_enc_1_A = digitalRead(ENCODER_1_A);
  sig_enc_1_B = digitalRead(ENCODER_1_B);
  sig_enc_2_A = digitalRead(ENCODER_2_A);
  sig_enc_2_B = digitalRead(ENCODER_2_B);
  sig_enc_3_A = digitalRead(ENCODER_3_A);
  sig_enc_3_B = digitalRead(ENCODER_3_B);
  sig_enc_4_A = digitalRead(ENCODER_4_A);
  sig_enc_4_B = digitalRead(ENCODER_4_B);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_A), encoderReadA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_B), encoderReadB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_A), encoderReadA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_B), encoderReadB2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_3_A), encoderReadA3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_3_B), encoderReadB3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_4_A), encoderReadA4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_4_B), encoderReadB4, CHANGE);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(motor_4_Pub);
}

void motorStart(MOTORS motor, String direct, uint16_t pwm) {
  switch (motor)
  {
  case MOTOR_1:
    if (direct == "CWW") {
      analogWrite(MOTOR_1_PWM, pwm);
      digitalWrite(MOTOR_1_DIR, LOW);
      digitalWrite(MOTOR_1_SLP, HIGH);
    }
    else if (direct == "CW")
    {
      analogWrite(MOTOR_1_PWM, pwm);
      digitalWrite(MOTOR_1_DIR, HIGH);
      digitalWrite(MOTOR_1_SLP, HIGH);
    }
    else {
      digitalWrite(MOTOR_1_PWM, LOW);
      digitalWrite(MOTOR_1_DIR, LOW);
      digitalWrite(MOTOR_1_SLP, LOW);
    }
    break;
  case MOTOR_2:
    if (direct == "CWW") {
      analogWrite(MOTOR_2_PWM, pwm);
      digitalWrite(MOTOR_2_DIR, LOW);
      digitalWrite(MOTOR_2_SLP, HIGH);
    }
    else if (direct == "CW")
    {
      analogWrite(MOTOR_2_PWM, pwm);
      digitalWrite(MOTOR_2_DIR, HIGH);
      digitalWrite(MOTOR_2_SLP, HIGH);
    }
    else {
      digitalWrite(MOTOR_2_PWM, LOW);
      digitalWrite(MOTOR_2_DIR, LOW);
      digitalWrite(MOTOR_2_SLP, LOW);
    }
    break;
  case MOTOR_3:
    if (direct == "CWW") {
      analogWrite(MOTOR_3_PWM, pwm);
      digitalWrite(MOTOR_3_DIR, LOW);
      digitalWrite(MOTOR_3_SLP, HIGH);
    }
    else if (direct == "CW")
    {
      analogWrite(MOTOR_3_PWM, pwm);
      digitalWrite(MOTOR_3_DIR, HIGH);
      digitalWrite(MOTOR_3_SLP, HIGH);
    }
    else {
      digitalWrite(MOTOR_3_PWM, LOW);
      digitalWrite(MOTOR_3_DIR, LOW);
      digitalWrite(MOTOR_3_SLP, LOW);
    }
    break;
  case MOTOR_4:
    if (direct == "CWW") {
      analogWrite(MOTOR_4_PWM, pwm);
      digitalWrite(MOTOR_4_DIR, LOW);
      digitalWrite(MOTOR_4_SLP, HIGH);
    }
    else if (direct == "CW")
    {
      analogWrite(MOTOR_4_PWM, pwm);
      digitalWrite(MOTOR_4_DIR, HIGH);
      digitalWrite(MOTOR_4_SLP, HIGH);
    }
    else {
      digitalWrite(MOTOR_4_PWM, LOW);
      digitalWrite(MOTOR_4_DIR, LOW);
      digitalWrite(MOTOR_4_SLP, LOW);
    }
    break;
  default:
    break;
  } 
}

void loop() {
  // Record the time
  currentMillis = millis();
  enc_4_tick_count.data = enc_tick_4;
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    motor_4_Pub.publish( &enc_4_tick_count );
    nh.spinOnce();
  }
  
  motorStart(MOTOR_1, "CWW", 100);
  motorStart(MOTOR_2, "CWW", 0);
  motorStart(MOTOR_3, "CWW", 0);
  motorStart(MOTOR_4, "CWW", 0);
  /*
  Serial.print(enc_tick_1);
  Serial.print("\t");
  Serial.print(enc_tick_2);
  Serial.print("\t");
  Serial.print(enc_tick_3);
  Serial.print("\t");
  Serial.println(enc_tick_4);
  */
}
