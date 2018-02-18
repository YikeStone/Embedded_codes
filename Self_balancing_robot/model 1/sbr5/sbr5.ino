#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#define LTMDIR 7
#define RTMDIR 4
#define LTMPWM 6
#define RTMPWM 5
#define LTMINT 2
#define RTMINT 3
#define MPUINT 8
MPU6050 mpu;
int rn = 0, ln = 0;
float r1 = 0, l1 = 0;
float rv = 0, lv = 0;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
bool rDir = 0;
bool lDir = 0;
uint32_t t, t1, t2;
int turn = 0;
int lout = 0;
int rout = 0;
double cgcerr = 0;
volatile uint8_t rc = 0;
volatile uint8_t lc = 0;
volatile bool mpuInterrupt = false;
Quaternion q;
VectorFloat gravity;
int16_t gyro[3];
float gy;
float ypr[3];
int m = 30;

float cg = 0;
float cgoffset = 0;
float angs = 20.0;
float alpha = 0, kcg, kcgd, kcgi, kangp = 1.0, kangd = 10.0;
float angle = 0;float pangle = 0;
void motorControl();
void getYPR();
void calculate();

void setup() {
    Wire.begin();
    Serial.begin(57600);
    delay(10);
    mpuInit();
    //delay(15000);
    intInit();
    delay(10);
    getYPR();
    cgoffset = 3.2;
    pinMode(LTMINT, INPUT);
    pinMode(RTMINT, INPUT);
    pinMode(LTMDIR, OUTPUT);
    pinMode(RTMDIR, OUTPUT);
    pinMode(LTMPWM, OUTPUT);
    pinMode(RTMPWM, OUTPUT);
    analogWrite(RTMPWM,1);
    analogWrite(LTMPWM,1);
    t=millis();
    t1=millis();
    //motorControl( 100,100);
}
int out = 0;
void loop() {
                getYPR();
                t2 = millis() - t;
                odor();
                cgPID(t2);
                angle = cg - ypr[1];    
                if(abs(pangle-angle)<4){
                  out = anglePD() + t2 * alpha / 100.0;
                  pangle = angle;
                }
                rc = lc = 0;
                lout = out + turn;
                rout = out - turn;
                rDir = ( (rout > 0) ? 1 : 0);
                lDir = ( (lout > 0) ? 1 : 0);
                lout = abs( lout );
                rout = abs( rout );
                
                if(lout != 0)
                {
                  if( (lout > 0) && (lout < m) )lout = m;
                }
                
                if(rout != 0)
                {
                  if( (rout > 0) && (rout < m) )rout = m;
                }
                lout = constrain(lout, -200, 200);
                rout = constrain(rout, -200, 200);
                if(abs(angle) > 40)
                  motorControl( 0, 0);
                else
                  motorControl( rout, lout);

            Serial.print(out);
            Serial.print("\t");
            
            //Serial.print(", ");
            Serial.print(ypr[1]);
            Serial.print("\t");
            Serial.println(abs(angle));
            //Serial.print("\tra la\t");
            //Serial.print(ra);
            //Serial.print(",");
            //Serial.println(la);
              if(Serial.available()>2){
                switch (Serial.read())
                {
                case 'p':kangp = Serial.parseFloat();break;
                case 'i':kcgi = Serial.parseFloat();break;
                case 'a':kcg = Serial.parseFloat();break;
                case 'b':kcgd = Serial.parseFloat();break;
                case 'd':kangd = Serial.parseFloat();break;
                case 's':cg  = Serial.parseFloat();t = millis();break;
                case 't':angs = Serial.parseFloat();break;
                //case 'a':an=Serial.parseFloat();break;
                case 'm':m = Serial.parseFloat();break;
                }
              Serial.read();
              }
              
//Serial.print(asp);Serial.print("\t");Serial.print(ap);Serial.print("\t");Serial.print(li);Serial.print("\t");Serial.print(ad);Serial.print("\t");Serial.println(lp);
}

void cgPID(uint32_t t){
  float err = t * alpha / 100.0 - (rv + lv) / 2.0;
  cgcerr += kcgi * err;
  cgcerr = constrain( cgcerr, -5, 5);
  cg = err * kcg + cgcerr + kcgd * gy + cgoffset;
  cg = constrain(cg, -10, 10);
}

float anglePD()
{
  float err = alpha - angle;
  //return (err/abs(err))*(pow(kangp,abs(err)/57.2958)-1)*255.0/(pow(kangp,angs/57.2958)-1) + (gy) * kangd;
  return (err * abs(angle) * kangp + gy * kangd);
}

void odor(){
  t1 = (millis() - t1);
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
  rv = rv * 1000.0 / t1;
  lv = lv * 1000.0 / t1;
  t1=millis();
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

void getYPR(){
while (!mpuInterrupt && fifoCount < packetSize);
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) 
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetGyro(gyro, fifoBuffer);
            ypr[1] *= 180 / PI;
            gy = -gyro[1]/100.0;
     }
}

void intInit(){
    mpu.resetFIFO();
    TCCR1A=0;
    TCCR1B=0;
    //TCNT1=64886;
    TCCR1B |= (1 << CS12);
    TIMSK1 |= (1 << TOIE1);
    attachInterrupt(digitalPinToInterrupt(RTMINT),RME,RISING);
    attachInterrupt(digitalPinToInterrupt(LTMINT),LME,RISING);
}

void LME(){
  lc++;
}

void RME(){
  rc++;
}
ISR (PCINT0_vect){
  if(digitalRead(8) == 1)mpuInterrupt = 1;
}
void mpuInit(){
    mpu.initialize();
    pinMode(MPUINT, INPUT);
    mpu.testConnection();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(-1);    
    mpu.setYGyroOffset(-1);
    mpu.setZGyroOffset(255);
    mpu.setXAccelOffset(705);
    mpu.setYAccelOffset(-139);
    mpu.setZAccelOffset(1219);
    mpu.setDMPEnabled(true);
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        *digitalPinToPCMSK(MPUINT) |= bit (digitalPinToPCMSKbit(MPUINT));
        PCIFR  |= bit (digitalPinToPCICRbit(MPUINT));
        PCICR  |= bit (digitalPinToPCICRbit(MPUINT));
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
  }
