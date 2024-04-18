#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

#define PWMB 3
#define BIN2 5
#define BIN1 4  
#define STBY 6
#define AIN1 8
#define AIN2 7
#define PWMA 9

//PID
float Kp = 0.1398;//0.1398; //0.1398
float Ki = 0.0000;
float Kd = 0.01252;//0.01252; //0.01195

//BUTTON
int button_calibration = 11;

//SPEED
const uint8_t maxspeedA = 220 ;  //150; 
const uint8_t maxspeedB = 220 ; //150; 
const uint8_t basespeedA =150 ;   //100; 
const uint8_t basespeedB =150 ;   //100; 

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int lasterror = 0;
int I = 0; // Integration term for PID

void setup() {
  pinMode(PWMB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(button_calibration, INPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){19, 18, 17, 16, 15,14}, SensorCount);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  while (digitalRead(button_calibration) == LOW) {
    // Wait until calibration button is pressed
  }
  delay(300);

  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED 
  for (uint16_t i = 0; i < 100; i++) {
    motor1.drive(90);
    motor2.drive(-90);
    qtr.calibrate();
  }
  for (uint16_t i = 0; i <= 100; i++) {
    motor1.drive(-90);
    motor2.drive(90);
    qtr.calibrate();      
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED

  motor1.brake();
  motor2.brake();
  delay(200);

  // Wait until calibration button is released
  while (digitalRead(button_calibration) == LOW) {
    // Do nothing
  }
  delay(300);
}

void PID_Control() {
  uint16_t position = qtr.readLineBlack(sensorValues); 
  int error = 2500 - position; 
  int P = error;
  I += error; // Accumulate error over time
  int D = error - lasterror;
  lasterror = error;

  int motorspeed = P * Kp + I * Ki + D * Kd; 

  int motorspeedA = basespeedA + motorspeed;
  int motorspeedB = basespeedB - motorspeed;
  
  if (motorspeedA > maxspeedA) {
    motorspeedA = maxspeedA;
  }
  if (motorspeedB > maxspeedB) {
    motorspeedB = maxspeedB;
  }
  if (motorspeedA < 0) {
    motorspeedA = -90;
  }
  if (motorspeedB < 0) {
    motorspeedB = -90;
  } 
   
  mspeed(motorspeedA, motorspeedB);
}

void mspeed(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

void loop() {

  PID_Control();
}
