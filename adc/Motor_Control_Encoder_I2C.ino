
#define debug 1
///////////////////// Steering Servo Control /////////////////////
#define RC_SERVO_PIN 8
#define NEURAL_ANGLE 68
#define LEFT_STEER_ANGLE  -40
#define RIGHT_STEER_ANGLE  40

#include <Servo.h>
Servo   Steeringservo;
int Steering_Angle = NEURAL_ANGLE;

void steering_control()
{
  if(Steering_Angle<= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  Steeringservo.write(Steering_Angle);  
}

///////////////////// Steering Servo Control /////////////////////

/////////////////////// DC Motor Control /////////////////////
#include "src/PinChangeInt.h"

volatile long encoderPos = 0, last_pos=0;
unsigned long lastTime,now;
double input = 0;

#define MOTOR_PWM 5 
#define MOTOR_DIR 4
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      3                       // Quadrature encoder B pin

int Motor_Speed = 0;

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise  
}

void encoder()  
{                                              // pulse and direction, direct port reading to save cycles  
  //Serial.println("interrupt");
  if(digitalRead(encodPinB1)==LOW)    encoderPos--;             // if(digitalRead(encodPinB1)==HIGH)   count --;
  else                                 encoderPos++;             // if(digitalRead(encodPinB1)==LOW)   count ++;
}

void motor_control(int dir, int motor_pwm)
{
  
  if(dir == -1) // forward
  { 
    digitalWrite(MOTOR_DIR,LOW);
    analogWrite(MOTOR_PWM,motor_pwm);    
  }
  else if(dir == 1) // backward
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM,motor_pwm);
  }
  else // stop
  {
    analogWrite(MOTOR_PWM,0);
  }
}

/////////////////////// DC Motor Control /////////////////////

////////////////////// Sonar Sensor Setup ////////////////////
#include <NewPing.h>
#define TRIG 9
#define ECHO 10
#define Sonar_MAX_Distance 150
NewPing sonar(9,10,Sonar_MAX_Distance);
int sonar_d = 0;

void sonar_sensor_read(void)
{
  sonar_d=sonar.ping_cm();
  if(sonar_d == 0) sonar_d = Sonar_MAX_Distance;  
}

////////////////////// Sonar Sensor Setup ////////////////////

/////////////////////////// I2C 통신 //////////////////////////
#include <Wire.h>

#define debug 1

void receiveEvent(int howMany)
{
  unsigned  char a[6];
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();
  a[3] = Wire.read();
  a[4] = Wire.read();
  a[5] = Wire.read();


  Steering_Angle =  a[1]*256 + a[2];
  Motor_Speed    =  a[4]*256 + a[5];
 
 
  Serial.print(a[0]);   Serial.print("  ");  Serial.print(a[1]);  Serial.print("  ");
  Serial.print(a[2]);   Serial.print("  ");  Serial.print(a[4]);   Serial.print("  ");
  Serial.print(a[5]);   Serial.print("  ");
  Serial.print(Steering_Angle);   Serial.print("  ");
  Serial.println(Motor_Speed);

  steering_control();
   
  if(Motor_Speed> 0)      motor_control(1,Motor_Speed);
  else if(Motor_Speed< 0) motor_control(-1,-Motor_Speed);
  else motor_control(0,0);
   
}
/////////////////////////// I2C 통신 //////////////////////////

void requestEvent() 
{
  unsigned char s[6] = {0,};
  int temp;
  temp = sonar_d*10;
  
  //s[0]='#';
  
  s[0]= (temp&0xff00)>>8;
  s[1]= (temp&0x00ff); 

  /*
  s[2]= (encoderPos&0xff000000)>>24;    // encoder MSB 8bit
  s[3]= (encoderPos&0x00ff0000)>>16;
  s[4]= (encoderPos&0x0000ff00)>>8;
  s[5]= (encoderPos&0x000000ff);        // encoder LSB 8bit
  //s[7]='*'; 
  */

  s[2]= (encoderPos&0xff00)>>8;
  s[3]= (encoderPos&0x00ff);
  s[4] = 0;
  s[5] = 0;
  
  Wire.write(s,6); // respond 
  //sensor_flag = 0;
}

void setup()
{
  interrupt_setup();
  Wire.begin(5);    // I2C bus  #5
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(Steering_Angle);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(115200);
  delay(2000);
}

void loop()
{

  now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=500 )
  {
      //input = (360.0*1000*(encoderPos-last_pos)) /(1856.0*(now - lastTime));
      lastTime=now;
      last_pos=encoderPos;
  }
  sonar_sensor_read();
  if(debug == 1)
  {   
    ///////////// Steering Servo  ////////////// 
    Serial.print("Sonar : ");
    Serial.print(sonar_d);
    Serial.print("cm");    

    ///////////// Steering Servo  /////////////
    Serial.print("  Steering Angle : ");
    Serial.print(Steering_Angle);

    ////////////// encoder  ///////////////////
    Serial.print("  Encoder Pos : ");       
    Serial.print(encoderPos);
    
    /////////////// Motor PWM  ///////////////// 
    Serial.print("  Motor PWM : ");
    Serial.println(Motor_Speed);
  }
}
