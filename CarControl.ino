/*
 * Arduino IRCarControl
 * Time:Wed Feb  6 17:40:04 CST 2019
 * Author:January
 */
#include "IRremote.h"
#include "MotorDriver.h"
#include<Servo.h>

#define DEBUG

//remote control(IR)
#define RECV_PIN 11
#define PRESSING_DELAY 2
#define PRESSING 0xFF
//remote control bluetooth

//motor
#define BIN_ONE 8
#define BIN_TWO 7
#define PWMB 10
#define AIN_ONE 2
#define AIN_TWO 4
#define PWMA 9

//utralsonic 
#define UTRALSONIC_TRIGGER 13   
#define UTRALSONIC_SIG 12
#define SAFEDISTANCE 15 //cm
#define MAXDISTANCE 255

//servo
#define SERVO_PIN 9

//buzzer
#define BUZZER_PIN 3

//car_state
#define FORWARD 0
#define BACKWARD 1
#define TURNLEFT 2
#define TURNRIGHT 3
#define STOP 4

IRrecv irrecv(RECV_PIN);
decode_results result;
MotorDriver motor;
Servo servo;
int car_state;

//leverage utralsonic sensor to detect possible blocks
float detect(){
  int pulse;
  digitalWrite(UTRALSONIC_TRIGGER, HIGH);
  delayMicroseconds(11);
  digitalWrite(UTRALSONIC_TRIGGER, LOW);
  pulse = pulseIn(UTRALSONIC_SIG, HIGH, 10000);
  
#ifdef DEBUG
  Serial.print("barrier distance:");
  Serial.println(pulse * 0.034);
#endif
  if(pulse == 0){
    return MAXDISTANCE;
  }
  return 0.034 * pulse;  
}

void utralsonic_init(){
  pinMode(UTRALSONIC_TRIGGER, OUTPUT);
  pinMode(UTRALSONIC_SIG, INPUT);
  digitalWrite(UTRALSONIC_SIG, LOW);
}

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  motor.init(BIN_ONE, BIN_TWO, PWMB, AIN_ONE, AIN_TWO, PWMA);
  utralsonic_init();
  car_state = STOP;
  motor.setSpeed(255, MOTORA);
  motor.setSpeed(255, MOTORB);
  pinMode(BUZZER_PIN, OUTPUT);
  
  //servo.attach(SERVO_PIN);
}


void loop() {
  // put your main code here, to run repeatedly:
  int ir_code;
  int utral;
  static int filter = 0;
  
  if(irrecv.decode(&result)){
    filter = 0;   
    ir_code = (result.value & 0xFF00) >> 8;
#ifdef DEBUG 
    Serial.println(ir_code, HEX);
#endif
    switch(ir_code){
      case KEY_PLUS:
        if(detect() > SAFEDISTANCE){
          car_state = FORWARD;
          motor.goForward();
        }
        break;
      case KEY_MINUS:
        car_state = BACKWARD;
        motor.goBackward();
        break;
      case KEY_PREV:
      car_state = TURNLEFT;
        motor.goRight();
        break;
      case KEY_NEXT:
        car_state = TURNRIGHT;
        motor.goLeft();
        break;
      case KEY_POWER:
        car_state = STOP;
        motor.stop();
        break;
      case PRESSING:
        if(car_state == FORWARD && detect() < SAFEDISTANCE){
          car_state = STOP;
          motor.stop();
        }
        break;
      case KEY_ONE:
        digitalWrite(BUZZER_PIN, HIGH);
      default:
        car_state = STOP;
        motor.stop();
    }
    irrecv.resume();
  }else{
    filter++;
    if(filter > PRESSING_DELAY){
      car_state = STOP;
      motor.stop();
      digitalWrite(BUZZER_PIN, LOW);
      filter = 0;
    }
  }
  delay(50);
}
