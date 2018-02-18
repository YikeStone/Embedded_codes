#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <MadgwickAHRS.h>
MPU6050 mpu;
Madgwick filter;
#define RAD2DEG 57.2957795
#define MPUINT 20
#define LTMDIR 8
#define RTMDIR 11
#define LTMPWM 9
#define RTMPWM 10
#define LTMINT 6
#define RTMINT 7
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
int16_t Mag[3];
float mag[3];
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float acc[3];
float ypr[3];
volatile bool mpuInterrupt = false;
float displ = 0;
int rn = 0, ln = 0;
float r1 = 0, l1 = 0;
float rv = 0, lv = 0;
bool rDir = 0;
bool lDir = 0;
int turn = 0;
int lout = 0;
int rout = 0;
int mov = 0;
int16_t gyro[3];
float gy[3];
volatile uint8_t rc = 0;
volatile uint8_t lc = 0;
int m = 28;
float setPoint;
uint32_t t = 0;
uint16_t DT =0;
float angle;
float initAOffset = 4.7;
float RSP;
float steering;
float cg=0;
float speedFilter = 0;
uint32_t intT = 0, dt;
float out = 0;
void dmpDataReady() {
    mpuInterrupt = true;
}
void mpuInit(){
    mpu.initialize();
    mpu.testConnection();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(-1);    
    mpu.setYGyroOffset(-1);
    mpu.setZGyroOffset(255);
    mpu.setXAccelOffset(705);
    mpu.setYAccelOffset(-139);
    mpu.setZAccelOffset(1219);
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
}
void LME(){
  lc++;
}
void RME(){
  rc++;
}
//float K1=0.1,K2=0.18,K3=6.5,K4=1.12;
//float K1=0.0577,K2=0.0725,K3=5.4045,K4=0.6276;
//float K1=0,K2=0;
//float K1=31.6228,K2=41.4006, K3=942.3895,K4=84.9139;
//float K1=31.6,K2=84.6,K3=4184.1,K4=370.2;
//float K1=70.7,K2=131.5,K3=3837.7,K4=369.4;
//float K1=1000,K2=731,K3=30606,K4=3146;
float K1=0,K2=0.3,K3=664.3,K4=49.2;
//float Kp=11,Kd=7.0,Ki=0;float verr=0;
int getPWM(float a,float b,float c,float d){
  /*if(c>=0)return (pow(Kp,abs(c))-1)*255.0/(pow(Kp,20/57.2958)-1) + (Kd) * 7.0;
  c*=-1;
  verr+=b;
  return -(pow(Kp,abs(c))-1)*255.0/(pow(Kp,20/57.2958)-1) + (Kd) * 7.0;
  */
  return constrain((a*K1+b*K2+c*K3+d*K4),-200,200);
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
void motorControl(int a, int b)
{
    digitalWriteFast(LTMDIR, !lDir);
    digitalWriteFast(RTMDIR, !rDir);
    analogWrite(RTMPWM, a * 1.2 + 10);
    analogWrite(LTMPWM, b);
}
void setup() {
    pinMode(13,OUTPUT);
    digitalWrite(13,1);
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.begin(57600);
    pinMode(LTMINT, INPUT);
    pinMode(RTMINT, INPUT);
    pinMode(LTMDIR, OUTPUT);
    pinMode(RTMDIR, OUTPUT);
    pinMode(LTMPWM, OUTPUT);
    pinMode(RTMPWM, OUTPUT);
    pinMode(MPUINT, INPUT);
    digitalWrite(LTMINT,HIGH);
    digitalWrite(RTMINT,HIGH);
    digitalWrite(MPUINT,HIGH);
    mpuInit();
    filter.begin(100);
    attachInterrupt(digitalPinToInterrupt(MPUINT), dmpDataReady, RISING);
    attachInterrupt(digitalPinToInterrupt(RTMINT),RME,RISING);
    attachInterrupt(digitalPinToInterrupt(LTMINT),LME,RISING);
    initAOffset = 6.3;    
    t=millis();
}
void loop() {
    t=millis();
    Serial.print('a');
                //Serial.print(-displ);Serial.print(',');
                //Serial.print(-speedFilter);Serial.print(',');
                Serial.print(angle);Serial.println(',');
                for(int i=0;i<3;i++){
                  Serial.print("\tgyro: ");Serial.print(gyro[i]);Serial.print(',');
                  
                  }
                  for(int i=0;i<3;i++){
                    Serial.print("\tMAG: ");Serial.print(Mag[i]);Serial.print(',');
                    }
                    Serial.print("\tgravity: ");
                  Serial.print(gravity.x);Serial.print(',');
                  Serial.print(gravity.y);Serial.print(',');
                  Serial.print(gravity.z);Serial.print(',');
                //Serial.print(gy);Serial.print(',');
                //Serial.print(out);Serial.print(',');
                //Serial.println(DT);
    while (!mpuInterrupt && fifoCount < packetSize) {
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
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
        mpu.resetFIFO();    
    else if (mpuIntStatus & 0x02) 
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize; 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetMag(Mag, fifoBuffer);
        mag[0] = Mag[1] * 1200.0 / 32768.0;
        mag[1] = Mag[0] * 1200.0 / 32768.0;
        mag[2] = -Mag[2] * 1200.0 / 32768.0;
        acc[0] = gravity.x * 2.0 / 32768.0;
        acc[1] = gravity.x * 2.0 / 32768.0;
        acc[2] = gravity.x * 2.0 / 32768.0;
        gy[0] = gyro[0] * 250.0 / 32768.0;
        gy[1] = gyro[1] * 250.0 / 32768.0;
        gy[2] = gyro[2] * 250.0 / 32768.0;
        ypr[0] *= 180 / M_PI;
        ypr[1] *= 180 / M_PI;
        ypr[2] *= 180 / M_PI;
        
     }
     angle = ypr[1] - initAOffset;
     DT = millis() - t;
     odor();
     out = getPWM(displ,speedFilter+mov,angle/RAD2DEG,gy[1]);
     lout = out + steering;
                rout = out - steering;
                rDir = ( (rout > 0) ? 1 : 0);
                lDir = ( (lout > 0) ? 1 : 0);
                if(lout!=0)
                lout = abs( constrain(lout, -200, 200) ) + m;
                if(rout!=0)
                rout = abs( constrain(rout, -200, 200) ) + m;
                /*if(abs(angle)>70)motorControl(0,0);
                else
                motorControl( rout, lout);
*/}
