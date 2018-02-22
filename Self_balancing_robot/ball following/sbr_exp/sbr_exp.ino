#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
bool blinkState = false;
double ra=0;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t T=0;
uint16_t pT=0;
uint16_t dT=0;
float ITerm =0;
float DTerm =0;
float kp = 11;
float ki = 0.04;
float kd = 7.0;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
int16_t gg[3];
VectorFloat gravity;    // [x, y, z]            gravity vector
float Setpoint = 0;
float raSet=-10.0;
float setT=7.5;
float t=255.0/(pow(kp,setT/57.2958)-1);
int flag=0;
float disp=0;
float r=0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin();
        pinMode(10, OUTPUT);  
  pinMode(4, OUTPUT); 
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);  
  pinMode(7, OUTPUT);  
  pinMode(9, OUTPUT);
  digitalWrite(7,0);
  digitalWrite(4,0);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  analogWrite(10,1);
  analogWrite(9,1);
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    //delay(5000);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    mpu.testConnection(); 
    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-965);
    mpu.setYGyroOffset(-102);
    mpu.setZGyroOffset(35);
    mpu.setZAccelOffset(1220); // 1688 factory default for my test chip
    mpu.setXAccelOffset(-5082);
    mpu.setYAccelOffset(-739);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {Serial.println("Start");
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
    //OCR1A=OCR1B=255;
}


float error=0,output=0;
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetGyro(gg,fifoBuffer);
            if(Serial.available()>2)
            {
              //delay(3);
              switch(Serial.read()){
              case 's': raSet=Serial.parseFloat();break;
              case 'p': kp=Serial.parseFloat();t=255.0/(pow(kp,setT/57.2958)-1);break;
              case 'i': ki=Serial.parseFloat();break;
              case 'd': kd=Serial.parseFloat();break;
              case 'r': r=Serial.parseFloat();break;
              case 'a': setT=Serial.parseFloat();t=255.0/(pow(kp,setT/57.2958)-1);break; 
             }
             Serial.read();
            }
            //Serial.print("a  ");
            ra=atan2(gravity.z,gravity.x)*57.2958-raSet;
            Serial.println(ra+raSet);
            //Serial.print("\t");
            if(ra<0)flag = -1;
            else flag = 1;
            //Serial.print("a ");
            //Serial.print(ra,9);
            ra = flag*ra;
            //Serial.println();
            
            output = (pow(kp,constrain(ra,0,setT)/57.2958)-1)*flag*t;
            error = output - raSet;
            DTerm = error-DTerm;
            ITerm+= (ki * error);
            if(ITerm > 255) ITerm= 255;
            else if(ITerm < -255) ITerm= -255;
            //if((ra<raSet)&&(ra>-raSet))ITerm=0;
            //if((ITerm/abs(ITerm))!=(output/abs(output)))ITerm=error;
            DTerm *=kd;
            output=output+ITerm+DTerm;
            DTerm = error;
            /*if((ra>20.0)&&(ra<45))output=255*flag;
            if(ra>=45)output=0;*/
            if(output >=255) output= 255;
            else if(output <=-255) output= -255;
            
            //Serial.println(output);
            if (output > 0) PORTD=B01100000;
            else PORTD=B10010000; 
            
            if(abs(output)<7)
                  OCR1A=OCR1B=map(abs(output),0,255,7,255);
            else
               OCR1A=OCR1B=abs(output);
    }
}
