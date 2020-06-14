#include <AVR_keywords.h>

#define LINE_SENSOR_1 A7
#define LINE_SENSOR_2 A6
#define LINE_SENSOR_3 A5
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A3
#define LINE_SENSOR_6 A2
#define LINE_SENSOR_7 A1

#define  OFFSET 10

#define DISTANCE_SENSOR A0

#define PWM_L_PIN 12
#define PWM_R_PIN 13

#define MOTOR_L_PIN1 17
#define MOTOR_L_PIN2 18

#define MOTOR_R_PIN1 16
#define MOTOR_R_PIN2 19

#define THRESHOLD_LINE 500
#define THRESHOLD_DISTANCE 500


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
  analogWrite(PWM_R_PIN, PWM-2);
  
  digitalWrite(MOTOR_L_PIN1, LOW);
  digitalWrite(MOTOR_L_PIN2, HIGH);
  digitalWrite(MOTOR_R_PIN1, LOW);
  digitalWrite(MOTOR_R_PIN2, HIGH);
}

void smooth_turning(byte PWM_L, byte PWM_R){
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
long pozycja = 0;
long pozycja_last = 0;

double KP = 0.5;
double KD = 2.5;

double error_P = 0;
double error_D = 0;
int Error = 0;

void loop() {
//  go_straight(64);
//  delay(1000);
//  go_stop();
//  delay(1000);
long SENSOR1 = analogRead(LINE_SENSOR_1);
long SENSOR2 = analogRead(LINE_SENSOR_2);
long SENSOR3 = analogRead(LINE_SENSOR_3);
long SENSOR4 = analogRead(LINE_SENSOR_4);
long SENSOR5 = analogRead(LINE_SENSOR_5);
long SENSOR6 = analogRead(LINE_SENSOR_6);
long SENSOR7 = analogRead(LINE_SENSOR_7);

long suma = (SENSOR1 + SENSOR2 + SENSOR3 + SENSOR4 + SENSOR5 + SENSOR6 + SENSOR7);
long sr = (SENSOR1*-3*OFFSET + SENSOR2*-2*OFFSET + SENSOR3*-1*OFFSET + SENSOR4*0*OFFSET + SENSOR5*1*OFFSET + SENSOR6*2*OFFSET + SENSOR7*3*OFFSET);
pozycja = sr/suma;

error_P = pozycja * KP; // zależność liniowa
error_D = (pozycja - pozycja_last) * KD; // różnica pomiędzy pomiarami
Error = (int)(error_P + error_D); // suma członów regulatora
pozycja_last = pozycja;

//  Serial.print(error_P);
//  Serial.print("\t");
//  Serial.print(error_D);
//  Serial.print("\t");
//  Serial.print(pozycja);
//  Serial.print("\n");

if(Error > 0){
  smooth_turning(40 + Error, 40);
} else {
  smooth_turning(40, 40 - Error);
}

//
//  Serial.print(pozycja);
//  Serial.print("\t");
//  //Serial.println(SENSOR1*-3*OFFSET + SENSOR2*-2*OFFSET + SENSOR3*-1*OFFSET + SENSOR4*0*OFFSET + SENSOR5*1*OFFSET + SENSOR6*2*OFFSET + SENSOR7*3*OFFSET);
//  Serial.print("\n");

//  Serial.print(SENSOR1);
//  Serial.print("\t");
//  Serial.print(SENSOR2);
//  Serial.print("\t");
//  Serial.print(SENSOR3);
//  Serial.print("\t");
//  Serial.print(SENSOR4);
//  Serial.print("\t");
//  Serial.print(SENSOR5);
//  Serial.print("\t");
//  Serial.print(SENSOR6);
//  Serial.print("\t");
//  Serial.print(SENSOR7);
//  Serial.print("\t");
//  Serial.println(analogRead(LINE_SENSOR_7));
//  Serial.println("\t");
//  Serial.println(analogRead(DISTANCE_SENSOR));
//  if( SENSOR1 > THRESHOLD_LINE && SENSOR2 > THRESHOLD_LINE && SENSOR3 > THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 > THRESHOLD_LINE && SENSOR6 > THRESHOLD_LINE && SENSOR7 > THRESHOLD_LINE){
//    go_stop();
//  }
//  if( (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 > THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 > THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE) ||
//      (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 > THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 < THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE) ||
//      (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 < THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 > THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE) ||
//      (SENSOR1 < THRESHOLD_LINE && SENSOR2 < THRESHOLD_LINE && SENSOR3 < THRESHOLD_LINE && SENSOR4 > THRESHOLD_LINE && SENSOR5 < THRESHOLD_LINE && SENSOR6 < THRESHOLD_LINE && SENSOR7 < THRESHOLD_LINE)){
//    go_straight(30);
//  }else go_stop();

  delay(10);

}
