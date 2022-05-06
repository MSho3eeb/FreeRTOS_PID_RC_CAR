#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>

#define MinOutput -255
#define MaxOutput 155
#define KP 0.1
#define KI 0.5
#define KD 0.001
#define EncoderPin1 2
#define EncoderPin2 3
#define MotorSpeedPin1 5
#define MotorSpeedPin2 6
#define Motor1out1 A0
#define Motor1out2 A1
#define Motor2out1 12
#define Motor2out2 13
#define EncoderPulsePerRev 40
const TickType_t TimerPeriod = 1000 / portTICK_PERIOD_MS;
double error = 0;
double pulseCounter1;
double pulseCounter2;
double RPM;
double RPM1;
double motorPWM1 = 0;
double motorPWM2 = 0;
double SetPoint1 = 0;
double SetPoint2 = 0;
double revsPerUnit1 = 0;
double revsPerUnit2 = 0;
static volatile char command1;

void RPMCalc(void *pvParameters);
void theMain(void *pvParameters);

PID motor1SpeedPID1(&RPM, &motorPWM1, &SetPoint1, KP, KI, KD, DIRECT);
PID motor1SpeedPID2(&RPM1, &motorPWM2, &SetPoint2, KP, KI, KD, DIRECT);


void encoderISR1() {
  pulseCounter1++;
  //Serial.println("sssss1");
}
void encoderISR2() {
  pulseCounter2++;
  //Serial.println("sssss2");
}
void forward(){
  digitalWrite(Motor1out1, HIGH);
  digitalWrite(Motor1out2, LOW);
  digitalWrite(Motor2out1, HIGH);
  digitalWrite(Motor2out2, LOW);
  SetPoint1 = 250;
  SetPoint2 = 250;
}
void backward(){
  digitalWrite(Motor1out1, LOW);
  digitalWrite(Motor1out2, HIGH);
  digitalWrite(Motor2out1, LOW);
  digitalWrite(Motor2out2, HIGH);
  SetPoint1 = 250;
  SetPoint2 = 250;
}
void right(){
  digitalWrite(Motor1out1, LOW);
  digitalWrite(Motor1out2, HIGH);
  digitalWrite(Motor2out1, HIGH);
  digitalWrite(Motor2out2, LOW);
  SetPoint1 = 250;
  SetPoint2 = 250;
}
void left(){
  digitalWrite(Motor1out1, HIGH);
  digitalWrite(Motor1out2, LOW);
  digitalWrite(Motor2out1, LOW);
  digitalWrite(Motor2out2, HIGH);
  SetPoint1 = 250;
  SetPoint2 = 250;
}
void stopp(){
  SetPoint1 = 0;
  SetPoint2 = 0;
}


void setup() {
  Serial.begin(9600);
  pinMode(EncoderPin1, INPUT);
  pinMode(EncoderPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderPin1), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderPin2), encoderISR2, RISING);
  pinMode(Motor1out1, OUTPUT);
  pinMode(Motor1out2, OUTPUT);
  pinMode(MotorSpeedPin1, OUTPUT);
  pinMode(Motor2out1, OUTPUT);
  pinMode(Motor2out2, OUTPUT);
  pinMode(MotorSpeedPin2, OUTPUT);
  motor1SpeedPID1.SetMode(AUTOMATIC);
  motor1SpeedPID2.SetMode(AUTOMATIC);
  xTaskCreate(RPMCalc, "RPMCalc", 128, NULL, 2, NULL);
  xTaskCreate(theMain, "theMain", 128, NULL, 1 , NULL);
}

void loop() {
 //////////////////////////////////////////////////////////////////////////
}
void RPMCalc(void *pvParameters) {
  while (1) {
    revsPerUnit1 = (pulseCounter1 / 1161.0) * 250 * 60;
    revsPerUnit2 = (pulseCounter2 / 1161.0) * 250 * 60;
    //Serial.println(pulseCounter2);
    RPM = (revsPerUnit1 / 60);
    RPM1 = (revsPerUnit2 / 60);
    pulseCounter1 = 0;
    pulseCounter2 = 0;
    vTaskDelay(TimerPeriod);
  }
}
void theMain(void *pvParameters) {
  while (1) {
    motor1SpeedPID1.Compute();
    motor1SpeedPID2.Compute();
    analogWrite(MotorSpeedPin1, motorPWM1);
    analogWrite(MotorSpeedPin2, motorPWM2);
    while(Serial.available()>0){
      command1 = Serial.read();
    //Serial.println(command1);
      if(command1 == 'F'){
        forward();
      }else if(command1 == 'B'){
        backward();
      }else if(command1 == 'R'){
        right();
      }else if(command1 == 'L'){
        left();
      }else if(command1 == 'S'){
        stopp();
      }
    }  
  }
}
