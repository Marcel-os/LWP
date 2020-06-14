#define LINE_SENSOR_1 A7
#define LINE_SENSOR_2 A6
#define LINE_SENSOR_3 A5
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A3
#define LINE_SENSOR_6 A2
#define LINE_SENSOR_7 A1

#define DISTANCE_SENSOR A0

#define PWM_L_PIN 13
#define PWM_R_PIN 12

#define MOTOR_L_PIN1 17
#define MOTOR_L_PIN2 18

#define MOTOR_R_PIN1 16
#define MOTOR_R_PIN2 19


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

void go_straight(){
  analogWrite(PWM_L_PIN, 150);
  analogWrite(PWM_R_PIN, 150);
  
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



void loop() {
  go_straight();
  delay(1000);
  go_stop();
  delay(1000);

  Serial.print(analogRead(LINE_SENSOR_1));
  Serial.print("\t");
  Serial.print(analogRead(LINE_SENSOR_2));
  Serial.print("\t");
  Serial.print(analogRead(LINE_SENSOR_3));
  Serial.print("\t");
  Serial.print(analogRead(LINE_SENSOR_4));
  Serial.print("\t");
  Serial.print(analogRead(LINE_SENSOR_5));
  Serial.print("\t");
  Serial.print(analogRead(LINE_SENSOR_6));
  Serial.print("\t");
  Serial.print(analogRead(LINE_SENSOR_7));
  Serial.print("\t");
  Serial.println(analogRead(DISTANCE_SENSOR));
  
  delay(500);

}
