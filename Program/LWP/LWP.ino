#include <AVR_keywords.h>

#define LINE_SENSOR_1 A7
#define LINE_SENSOR_2 A6
#define LINE_SENSOR_3 A5
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A3
#define LINE_SENSOR_6 A2
#define LINE_SENSOR_7 A1

#define  OFFSET 22

#define DISTANCE_SENSOR A0

#define PWM_L_PIN 12
#define PWM_R_PIN 13

#define MOTOR_L_PIN1 17
#define MOTOR_L_PIN2 18

#define MOTOR_R_PIN1 16
#define MOTOR_R_PIN2 19

#define THRESHOLD_LINE 500
#define THRESHOLD_DISTANCE 250

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_L_PIN1, OUTPUT);
  pinMode(MOTOR_L_PIN2, OUTPUT);
  pinMode(MOTOR_R_PIN1, OUTPUT);
  pinMode(MOTOR_R_PIN2, OUTPUT);

  digitalWrite(MOTOR_L_PIN1, LOW);
  digitalWrite(MOTOR_L_PIN2, LOW);
  digitalWrite(MOTOR_R_PIN1, LOW);
  digitalWrite(MOTOR_R_PIN2, LOW);

  analogWrite(PWM_L_PIN, 0);
  analogWrite(PWM_R_PIN, 0);
  //analogReference(INTERNAL);
}

void go_straight(byte PWM){
  analogWrite(PWM_L_PIN, PWM);
  analogWrite(PWM_R_PIN, PWM);
  
  digitalWrite(MOTOR_L_PIN1, LOW);
  digitalWrite(MOTOR_L_PIN2, HIGH);
  digitalWrite(MOTOR_R_PIN1, LOW);
  digitalWrite(MOTOR_R_PIN2, HIGH);
}

void smooth_turning(int PWM_L, int PWM_R){
  if(PWM_L > 255) PWM_L = 255;
  if(PWM_R > 255) PWM_R = 255;
//  if(PWM_L < 0) PWM_L = 0;
//  if(PWM_R < 0) PWM_R = 0;
  
//  Serial.print(PWM_L);
//  Serial.print("\t");
//  Serial.print(PWM_R);
//  Serial.print("\n");
  
  if(PWM_L < 0){
    PWM_L = -PWM_L;
    PWM_L /= 2;
    digitalWrite(MOTOR_L_PIN1, LOW);
    digitalWrite(MOTOR_L_PIN2, HIGH);
    digitalWrite(MOTOR_R_PIN1, HIGH);
    digitalWrite(MOTOR_R_PIN2, LOW);
    return;
  }
  if(PWM_R < 0){
    PWM_R = -PWM_R;
    PWM_R /= 2;
    digitalWrite(MOTOR_L_PIN1, HIGH);
    digitalWrite(MOTOR_L_PIN2, LOW);
    digitalWrite(MOTOR_R_PIN1, LOW);
    digitalWrite(MOTOR_R_PIN2, HIGH);
    return;
  }
  
  analogWrite(PWM_L_PIN, PWM_L);
  analogWrite(PWM_R_PIN, PWM_R);
  
  digitalWrite(MOTOR_L_PIN1, LOW);
  digitalWrite(MOTOR_L_PIN2, HIGH);
  digitalWrite(MOTOR_R_PIN1, LOW);
  digitalWrite(MOTOR_R_PIN2, HIGH);
}

void go_stop(){
  analogWrite(PWM_L_PIN, 255);
  analogWrite(PWM_R_PIN, 255);

  digitalWrite(MOTOR_L_PIN1, LOW);
  digitalWrite(MOTOR_L_PIN2, LOW);
  digitalWrite(MOTOR_R_PIN1, LOW);
  digitalWrite(MOTOR_R_PIN2, LOW);
}

void rotateR(byte PWM){

  analogWrite(PWM_L_PIN, PWM);
  analogWrite(PWM_R_PIN, PWM);
  
  digitalWrite(MOTOR_L_PIN1, HIGH);
  digitalWrite(MOTOR_L_PIN2, LOW);
  digitalWrite(MOTOR_R_PIN1, LOW);
  digitalWrite(MOTOR_R_PIN2, HIGH);

  delay(390);
  go_stop();
}

void rotateL(byte PWM){

  analogWrite(PWM_L_PIN, PWM);
  analogWrite(PWM_R_PIN, PWM);
  
  digitalWrite(MOTOR_L_PIN1, LOW);
  digitalWrite(MOTOR_L_PIN2, HIGH);
  digitalWrite(MOTOR_R_PIN1, HIGH);
  digitalWrite(MOTOR_R_PIN2, LOW);

  delay(450);
  go_stop();
}

void go_around(){
  if(analogRead(DISTANCE_SENSOR) >= THRESHOLD_DISTANCE + 20 ){
      go_stop();
  
  analogWrite(PWM_L_PIN, 50);
  analogWrite(PWM_R_PIN, 50);

  rotateR(50);
  go_straight(50);
  delay(1000);
  rotateL(50);
  go_straight(50);
  delay(1400);
  rotateL(50);
  go_stop();
  while( analogRead(LINE_SENSOR_4) < THRESHOLD_LINE ){
    go_straight(50);
    delay(10);
  }
  go_stop();
  //go right 90*
  //go straight
  //go left 90*
  //go straight
  //go left 90*
  //go straight
  }
  return;
}

long pozycja = 0;
long pozycja_last = 0;

double KP = 2.5;
double KD = 12.5;

double error_P = 0;
double error_D = 0;
int Error = 0;

void loop() {
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long SENSOR1 = analogRead(LINE_SENSOR_1);
long SENSOR2 = analogRead(LINE_SENSOR_2);
long SENSOR3 = analogRead(LINE_SENSOR_3);
long SENSOR4 = analogRead(LINE_SENSOR_4);
long SENSOR5 = analogRead(LINE_SENSOR_5);
long SENSOR6 = analogRead(LINE_SENSOR_6);
long SENSOR7 = analogRead(LINE_SENSOR_7);

long DISTANCE = analogRead(DISTANCE_SENSOR);
//  Serial.print(DISTANCE);
//  Serial.print("\n");

long suma = (SENSOR1 + SENSOR2 + SENSOR3 + SENSOR4 + SENSOR5 + SENSOR6 + SENSOR7);
long sr = (SENSOR1*-6*OFFSET + SENSOR2*-3*OFFSET + SENSOR3*-1*OFFSET + SENSOR4*0*OFFSET + SENSOR5*1*OFFSET + SENSOR6*3*OFFSET + SENSOR7*6*OFFSET);
pozycja = sr/suma;

byte VEL = 175;

error_P = pozycja * KP; // zaleĹĽnoĹ›Ä‡ liniowa
error_D = (pozycja - pozycja_last) * KD; // rĂłĹĽnica pomiÄ™dzy pomiarami
Error = (int)(error_P + error_D); // suma czĹ‚onĂłw regulatora
pozycja_last = pozycja;

if( abs(Error) <= 10 ){
  VEL += 60;
  //if( abs(Error) <= 5 )VEL += 100;
}

if( abs(Error) >= 25 ){
  VEL -= 45;
}

if(DISTANCE > THRESHOLD_DISTANCE){
  go_stop();
//  go_around();
}else{
  if(Error > 0){
  smooth_turning(VEL - Error, VEL + Error);
  }else {
  smooth_turning(VEL - Error, VEL + Error);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  if( (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 > THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 > THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE) ||
//      (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 > THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 < THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE) ||
//      (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 < THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 > THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE) ||
//      (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 < THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 < THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE)){
//    go_straight(30);
//  }else go_stop();

  delay(10);
}
