#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#define RAD2GRAD 57.2957795
#define LTMDIR 7
#define RTMDIR 4
#define LTMPWM 6
#define RTMPWM 5
#define LTMINT 2
#define RTMINT 3
MPU6050 mpu;
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
float oldRSP, RSP;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float Kp=0;
float Kd=0;

float Kp_thr=0;
float Ki_thr=0;
float speedFilter = 0;
uint32_t intT = 0, dt;
float out = 0;
void setup() {
    Wire.begin();
    Serial.begin(57600);
    delay(10);
    mpuInit();
    delay(15000);
    mpu.resetFIFO();
    initAOffset = 3.7;
    while(digitalRead(12)==0);
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
    intT = dt = millis();
}
void loop() {
            if(mpuInterrupt)
            {
                getYPR();
                DT = millis() - t;
                Serial.println(millis() - intT);
                if((millis()-intT) > 12){intT = millis();goto x;}
                intT = millis();
                angle = initAOffset - ypr[1];
                
                if((dt=millis()-dt)>50)
                {
                  odor();
                  target_angle = constrain(speedPIControl(speedFilter,throttle,Kp_thr,Ki_thr),-20.0,20.0);
                  dt = millis();
                }
                out = stabilityPDControl(angle,target_angle,Kp,Kd);  
                out = constrain(out,-255,255);
                lout = out + steering;
                rout = out - steering;
                rDir = ( (rout > 0) ? 1 : 0);
                lDir = ( (lout > 0) ? 1 : 0);
                
                if(lout!=0)
                lout = abs( constrain(lout, -255, 255) ) + m;
                if(rout!=0)
                rout = abs( constrain(rout, -255, 255) ) + m;
                
                if(abs(angle)>70)motorControl(0,0);
                else
                motorControl( rout, lout);
                t=millis();
                Serial.print(angle);Serial.print('\t');
                Serial.print(target_angle);Serial.print('\t');
                Serial.println(out);
            }x:;
            if(Serial.available()>2){
                switch (Serial.read())
                {
                case 'p':Kp = Serial.parseFloat();break;
                case 'i':Ki_thr = Serial.parseFloat();break;
                case 'd':Kd = Serial.parseFloat();break;
                case 'a':Kp_thr = Serial.parseFloat();break;
                //case 'b':kcgi = Serial.parseFloat();break;
                //case 'c':kcgd = Serial.parseFloat();break;
                case 's':initAOffset  = Serial.parseFloat();break;
                //case 'e':alpha  = Serial.parseFloat();t = millis();break;
                //case 't':angs = Serial.parseFloat();z = (pow(kangp,angs/57.2958)-1);break;
                //case 'a':an=Serial.parseFloat();break;
                case 'm':m = Serial.parseFloat();break;
                }
              Serial.read();
              }
}
float speedPIControl(float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;
  error = setPoint-input;
  PID_errorSum += constrain(error,-40,40);
  PID_errorSum = constrain(PID_errorSum,-4000,4000);
  output = Kp*error + Ki*PID_errorSum*dt*0.001;
  return(output);
}
float stabilityPDControl(float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;
  error = setPoint - input;
  //output = Kp*error + (Kd*(setPoint - setPointOld) - Kd*(input - PID_errorOld2))/DT;
  output = 400.0 * (1.0 / (1+exp(-error * Kp))-0.5) + (-Kd*(setPoint - setPointOld) + Kd*(input - PID_errorOld))/DT;
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;
  setPointOld = setPoint;
  return output;
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
  rv = rv / dt;
  lv = lv / dt;
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
  RSP = ( rv + lv ) / 2.0;
  speedFilter = RSP;
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
