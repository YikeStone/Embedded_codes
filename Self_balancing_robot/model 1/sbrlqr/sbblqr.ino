#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#define RAD2GRAD 57.2957795
#define RAD2M 0.20734511513
#define LTMDIR 7
#define RTMDIR 4
#define LTMPWM 6
#define RTMPWM 5
#define LTMINT 2
#define RTMINT 3
MPU6050 mpu;
float displ = 0;
int rn = 0, ln = 0;
float r1 = 0, l1 = 0;
float rv = 0, lv = 0;
uint16_t packetSize; 
uint16_t fifoCount;  
uint8_t fifoBuffer[64];
bool rDir = 0;
bool lDir = 0;
int turn = 0;
int lout = 0;
int rout = 0;
int mov = 0;
int16_t gyro[3];
float gy;
volatile uint8_t rc = 0;
volatile uint8_t lc = 0;
volatile bool mpuInterrupt = false;
Quaternion q;
VectorFloat gravity;
float ypr[3];
int m = 25;
float setPoint;
uint32_t t = 0;
uint16_t DT =0;
float angle;
float initAOffset = 4.7;
float RSP;
float steering;
float speedFilter = 0;
uint32_t intT = 0, dt;
float out = 0;
void setup() {
    Wire.begin();
    Serial.begin(57600);
    delay(10);
    mpuInit();
    //delay(15000);
    mpu.resetFIFO();
    initAOffset = 4.7;
    //while(digitalRead(12)==0);
    mpu.resetFIFO();
    intInit();
    while(!mpuInterrupt);
    getYPR();
    t=millis();
    pinMode(LTMINT, INPUT);
    pinMode(RTMINT, INPUT);
    pinMode(LTMDIR, OUTPUT);
    pinMode(RTMDIR, OUTPUT);
    pinMode(LTMPWM, OUTPUT);
    pinMode(RTMPWM, OUTPUT);
    analogWrite(RTMPWM,1);
    analogWrite(LTMPWM,1);
    intT = millis();
}
void odor(){
  if(!lDir)
    {
      lv = -1 * lc;
      lc = 0;
    }
  else
    {
      lv = lc;
      lc = 0;
    }
  if(!rDir)
    {
      rv = -1 * rc;
      rc = 0;
    }
  else
    {
      rv = rc;
      rc=0;
    }
  r1 += rv;
  l1 += lv;
  if(abs(r1) > 29)
  {
    if(r1 > 29) 
    {
      rn++;
      r1 -= 30;
    }
    else 
    {
      rn--;
      r1 += 30;
    }
  }
  if(abs(l1) > 29)
  {
    if(l1 > 29) 
    {
      ln++;
      l1 -= 30;
    }
    else 
    {
      ln--;
      l1 += 30;
    }
  }
  RSP = ( rv + lv ) * 3.45575191 / DT; // RAD2M / 60.0
  speedFilter = speedFilter*0.9 + RSP*0.1;
  displ = ( rn + ln + ( r1 + l1 ) / 30.0 ) * 0.10367255756; //RAD2M / 2.0
}

void LME(){
  lc++;
}

void RME(){
  rc++;
}
void mpuInit(){
    mpu.initialize();
    mpu.testConnection();
    mpu.dmpInitialize();
    mpu.setXGyroOffset(-1);    
    mpu.setYGyroOffset(-1);
    mpu.setZGyroOffset(255);
    mpu.setXAccelOffset(705);
    mpu.setYAccelOffset(-139);
    mpu.setZAccelOffset(1219);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  void getYPR(){
    mpuInterrupt = false;
    fifoCount = mpu.getFIFOCount();
        while (fifoCount < packetSize) {fifoCount = mpu.getFIFOCount();}
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            ypr[1] *= 180 / PI;
            mpu.dmpGetGyro(gyro, fifoBuffer);
            gy = gyro[1]/100.0;
            mpu.resetFIFO();
}


  
void motorControl(int a, int b)
{
    if(lDir)
      PORTD |= _BV(RTMDIR);
    else
      PORTD &= ~_BV(RTMDIR);
    if(rDir)
      PORTD |= _BV(LTMDIR);
    else
      PORTD &= ~_BV(LTMDIR);
    OCR0B = ( ( abs( a ) <= 200 ) ? a : 0 );
    OCR0A = ( ( abs( b ) <= 200 ) ? b : 0 ) * 1.2 + 10;
}

ISR(TIMER1_OVF_vect) {
  mpuInterrupt = 1;
  TCNT1=64886;
}

void intInit(){
    mpu.resetFIFO();
    TCCR1A=0;
    TCCR1B=0;
    TCNT1=64886;
    TCCR1B |= (1 << CS12);
    TIMSK1 |= (1 << TOIE1);
    attachInterrupt(digitalPinToInterrupt(RTMINT),RME,RISING);
    attachInterrupt(digitalPinToInterrupt(LTMINT),LME,RISING);
}
//float K1=0.1,K2=0.18,K3=6.5,K4=1.12;
//float K1=0.0577,K2=0.0725,K3=5.4045,K4=0.6276;
//float K1=31.6228,K2=41.4006,K3=942.3895,K4=84.9139;
float K1=1000,K2=731,K3=30606,K4=3146;
int getPWM(float a,float b,float c,float d){
  return constrain((a*K1+b*K2+c*K3+d*K4)/20.0,-200,200);
  }
/*
float K1=63.5894,K2=37.6050,K3=119.6250,K4=24.0648;
int getPWM(float a,float b,float c,float d){
  return -constrain((a*K1+b*K2+c*K3+d*K4)*0.1,-200,200);
  }
  */
void loop(){
  if(mpuInterrupt)
            {
                getYPR();
                DT = millis() - t;
                if((millis()-intT) > 12){intT = millis();goto x;}
                intT = millis();
                angle = ypr[1] - initAOffset;                
                odor();
                out = getPWM(displ,speedFilter+mov,angle/RAD2GRAD,gy/RAD2GRAD);
                lout = out + steering;
                rout = out - steering;
                rDir = ( (rout > 0) ? 1 : 0);
                lDir = ( (lout > 0) ? 1 : 0);
                
                if(lout!=0)
                lout = abs( constrain(lout, -200, 200) ) + m;
                if(rout!=0)
                rout = abs( constrain(rout, -200, 200) ) + m;
                
                if(abs(angle)>70)motorControl(0,0);
                else
                motorControl( rout, lout);
                t=millis();
                Serial.print('a');
                Serial.print(-displ);Serial.print(',');
                Serial.print(-speedFilter);Serial.print(',');
                Serial.print(angle);Serial.print(',');
                Serial.print(gy);Serial.print(',');
                Serial.print(out);Serial.println(',');
                //Serial.println(out);
            }x:;
            if(Serial.available()>2){
                switch (Serial.read())
                {
                case 'p':K1 = Serial.parseFloat();break;
                case 'i':K2 = Serial.parseFloat();break;
                case 'd':K3 = Serial.parseFloat();break;
                case 'a':K4 = Serial.parseFloat();break;
                /*case 'b':kcgi = Serial.parseFloat();break;
                case 'c':kcgd = Serial.parseFloat();break;*/
                case 's':initAOffset  = Serial.parseFloat();break;
                case 'e':mov  = Serial.parseInt();break;
                //case 't':angs = Serial.parseFloat();break;
                //case 'a':an=Serial.parseFloat();break;
                case 'm':m = Serial.parseFloat();break;
                }
              Serial.read();
              }
  }
