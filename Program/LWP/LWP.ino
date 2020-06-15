#include <AVR_keywords.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define LINE_SENSOR_1 A7
#define LINE_SENSOR_2 A6
#define LINE_SENSOR_3 A5
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A3
#define LINE_SENSOR_6 A2
#define LINE_SENSOR_7 A1
#define DISTANCE_SENSOR A0

#define PWM_L_PIN 12
#define PWM_R_PIN 13

#define MOTOR_L_PIN1 17
#define MOTOR_L_PIN2 18

#define MOTOR_R_PIN1 16
#define MOTOR_R_PIN2 19

#define KP 2
#define KD 16

#define SCALE_READINGS 22
#define THRESHOLD_LINE 500
#define THRESHOLD_DISTANCE 250
#define AVERAGE_SPEED 185

unsigned long ACTUAL_TIME = 0;
unsigned long SAVED_TIME = 0;
unsigned long DELTA_TIME = 0;

long pozycja = 0;
long pozycja_last = 0;

void go_straight(byte PWM);
void smooth_turning(int PWM_L, int PWM_R);
void go_stop();
void rotateR(byte PWM);
void rotateL(byte PWM);
void search_rotateL(byte PWM);
void search_rotateR(byte PWM);
void go_around();
void search_line(int err);

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
}

void loop(){
  
  int error_P = 0;
  int error_D = 0;
  int Error = 0;
  
  ACTUAL_TIME = millis();
  DELTA_TIME = ACTUAL_TIME - SAVED_TIME;
  
  if (DELTA_TIME >= 10UL) {
    SAVED_TIME = ACTUAL_TIME;
    
    long SENSOR1 = analogRead(LINE_SENSOR_1);        //Odczyt czujników lini
    long SENSOR2 = analogRead(LINE_SENSOR_2);
    long SENSOR3 = analogRead(LINE_SENSOR_3);
    long SENSOR4 = analogRead(LINE_SENSOR_4);
    long SENSOR5 = analogRead(LINE_SENSOR_5);
    long SENSOR6 = analogRead(LINE_SENSOR_6);
    long SENSOR7 = analogRead(LINE_SENSOR_7);
    
    long DISTANCE = analogRead(DISTANCE_SENSOR);     //Odczyt czujnika odległości
    
    long suma = (SENSOR1 + SENSOR2 + SENSOR3 + SENSOR4 + SENSOR5 + SENSOR6 + SENSOR7);
    long sr = (SENSOR1*-6*SCALE_READINGS + SENSOR2*-3*SCALE_READINGS + SENSOR3*-1*SCALE_READINGS + SENSOR4*0*SCALE_READINGS + SENSOR5*1*SCALE_READINGS + SENSOR6*3*SCALE_READINGS + SENSOR7*6*SCALE_READINGS);
    pozycja = sr/suma;                               //Wyznaczenie aktualnej pozycji robota
    
    error_P = pozycja * KP;                          //Wyliczenie członu P - proporcjonalego
    error_D = (pozycja - pozycja_last) * KD;         //Wyliczenie członu D - różniczkującego
    Error = error_P + error_D;                       //Suma P + D, czyli regulator PD
    pozycja_last = pozycja;
   
    int VEL = AVERAGE_SPEED ;                        //Redukacja oraz wzmocnienie prędkości bazowej na podstawie aktualnego uchybu uchybu.
    if( abs(Error) >= 25 ) VEL -= 45;
        else VEL += -6*abs(Error) + 100;

    if(DISTANCE > THRESHOLD_DISTANCE){
      go_stop(); 
      //go_around();                                 //Funkcja odpowiedzialna za omijanie przeszkód nie działa przy większych prędkościach ze względu na zbyt mały zakres pomiarowy czujnika odległości, działa poprawnie przy VEL <= 50.
    }else if( suma < 700 ){
      search_line(Error);                            //Funkcja poszukująca lini po wyjechaniu poza nią wszytskimi czujnikami, skręca robotem w stronę z której ostatnio była linia.
    }else if(suma > 5000){                                  
      go_stop();                                     //Jeżeli nie widzi nic zatrzymuje się.
    }else smooth_turning(VEL - Error, VEL + Error);  //Jeżeli widzi odpowiednio linie jedzie po niej.
  }
}

void smooth_turning(int PWM_L, int PWM_R){
  if(PWM_L > 255) PWM_L = 255;
  if(PWM_R > 255) PWM_R = 255;
  
  //gdy pwm < 0 odwróć kierunek obrotu koła
  if(PWM_L < 0){
    PWM_L /= -2;
    digitalWrite(MOTOR_L_PIN1, LOW);
    digitalWrite(MOTOR_L_PIN2, HIGH);
    digitalWrite(MOTOR_R_PIN1, HIGH);
    digitalWrite(MOTOR_R_PIN2, LOW);
    return;
  }
  if(PWM_R < 0){
    PWM_R /= -2;
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

void search_rotateL(byte PWM){

  analogWrite(PWM_L_PIN, PWM);
  analogWrite(PWM_R_PIN, PWM);
  
  digitalWrite(MOTOR_L_PIN1, LOW);
  digitalWrite(MOTOR_L_PIN2, HIGH);
  digitalWrite(MOTOR_R_PIN1, HIGH);
  digitalWrite(MOTOR_R_PIN2, LOW);
}

void search_rotateR(byte PWM){

  analogWrite(PWM_L_PIN, PWM);
  analogWrite(PWM_R_PIN, PWM);
  
  digitalWrite(MOTOR_L_PIN1, HIGH);
  digitalWrite(MOTOR_L_PIN2, LOW);
  digitalWrite(MOTOR_R_PIN1, LOW);
  digitalWrite(MOTOR_R_PIN2, HIGH);
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
  }
  return;
}

void search_line(int err){
  
  while( analogRead(LINE_SENSOR_4) < THRESHOLD_LINE ){
    if( err > 0 )  search_rotateR(40);
      else search_rotateL(40);
    if(analogRead(DISTANCE_SENSOR) >= THRESHOLD_DISTANCE + 20 ) {
      go_stop();
      return;
    }
  }
  go_stop();
  delay(5);
  return;
}

