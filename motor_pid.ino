#include <PID_v1.h>

#define pot  A0
#define en1  9
#define en2 10
#define pwm 5
#define sensor 2

float temp;
float last=0;
volatile double increment=0;

double Kp=1.07, Ki=3.15, Kd=.0426;
double Setpoint = 0, Input = 0, Output;

PID benandkevinsSMALLPPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(sensor, INPUT);
  
  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  
  attachInterrupt(digitalPinToInterrupt(sensor), counter, RISING);

  benandkevinsSMALLPPID.SetMode(AUTOMATIC);
}

void loop() {

  //Setpoint=analogRead(pot)/4.016;
  
  if( millis() - last > 100)
  {  
    double rpm = calculate_rpm();
    Serial.print(rpm); 
    Serial.print(", "); 
    Serial.println(map(analogRead(pot),0,1024,0,100));
    Input = map(rpm,0,90,0,255);
    //Input = map(calculate_rpm()/5000,0,1,0,255);
  }
  
  Setpoint = map(analogRead(pot),0,1024,0,255);
  benandkevinsSMALLPPID.Compute();
  analogWrite(pwm, Output);
  
 // analogWrite(pwm, Output);
}

void counter()
{
  ++increment;
}

double calculate_rpm()
{
  double rpm = increment / 30* 1000 / (millis()-last);
  increment = 0;
  last = millis();
  return rpm;
}
