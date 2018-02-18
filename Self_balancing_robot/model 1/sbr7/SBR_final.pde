#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Kalman.h"
MPU6050 mpu;
#define HMC5883L 0x1E
#define LTM_DIR 3
#define RTM_DIR 4
#define LTM_PWM 6
#define RTM_PWM 5
#define LF 1
#define RF 0
#define MPU_INT 2
#define rW 0.127
#define g 9.8
#define Ip 0.021
#define Iw 0.00242

bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float yaw[2];
float pitch[2];
Kalman filter;
uint32_t timer;
double RV, LV;
uint16_t rPWM;
uint16_t lPWM;
double RD, LD;
float yawOffset = 0.0;
float pitchOffset = 0;
uint8_t mode = 1;/*
				 0 - Calculate Offset.
				 1 - Maintain Balance and Position
				 2 - Turn by x degrees
				 3 - Move x distance
				 4 - Turn while moving
				 default - Maintain and Rotate
				 */

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

void setup() {
	Serial3.begin(57600);
	delay(100);
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	mpu.initialize();
	pinMode(MPU_INT, INPUT);
	pinMode(LTM_DIR, OUTPUT);
	pinMode(RTM_DIR, OUTPUT);
	pinMode(LTM_PWM, OUTPUT);
	pinMode(RTM_PWM, OUTPUT);
	mpu.testConnection();
	devStatus = mpu.dmpInitialize();
	/*mpu.setXGyroOffset(-1);
	mpu.setYGyroOffset(-1);
	mpu.setZGyroOffset(255);
	mpu.setXAccelOffset(705);
	mpu.setYAccelOffset(-139);
	mpu.setZAccelOffset(1219);
	*/
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);
	mpu.setDMPEnabled(true);
	attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();
	mpu.setI2CBypassEnabled(true);
	I2Cdev::writeByte(HMC5883L, 0x00,(0x03 << 5) |(0x04 << 2) |0x00 );
	I2Cdev::writeByte(HMC5883L, 0x01, 0x01 << 5);
	I2Cdev::writeByte(HMC5883L, 0x02, 0x01);
	setMode(1);delay(5000);
	while (pitch[0] != 0)readSensors();
	//delay(15000);
	TCCR4A = 0;
	TCCR4B = 0;
	TCCR4A |= (1 << COM4A0);
	TCCR4B = 0;
	TCCR4B |= (1 << WGM42);
	

	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3A |= (1 << COM3A0);
	TCCR3B = 0;
	TCCR3B |= (1 << WGM32);
}

void calculateOffset()
{


}

float pKpss = 0.5, pKdss = 4.5, pKiss = 0, yKpss = 1.0, yKdss = 0, dKpss, dKdss, yawcerr, yKiss = 0, preV, V, cPErr, vKiss = 5, vOffset = 0;
void stayStill() {
	float yawErr = yawOffset - yaw[0];
	float pitchErr = pitchOffset - pitch[0];
	cPErr += pitchErr;
	cPErr = constrain(cPErr, 500,500);
	RD += 0.001 * RV;
	LV = RV = ( pKpss * pitchErr + pKdss * pitch[1] + pKiss * cPErr);

	Serial3.print('a');
	Serial.print('\t');
	Serial3.print(pitch[0]);
	Serial3.print(',');
	Serial3.print(pitch[1]);
	Serial3.print(',');
	Serial3.print(LV);
	Serial3.print(',');
	Serial3.print(pitchOffset);
	Serial3.print(',');
	Serial3.print(RD);
	Serial3.println(',');

	//LV = RV = pKpss * pitchErr + pKdss * pitch[1] + pKiss * cPErr;
	/*if(pitchErr > 0)
		LV = RV = yKpss * pow(pKpss, abs(pitchErr)) + pKdss * pitch[1] + pKiss * cPErr;
	else 
		LV = RV = -yKpss * pow(pKpss, abs(pitchErr)) + pKdss * pitch[1] + pKiss * cPErr;
    */
	//LV -= yawErr*0.1;
	//RV += yawErr*0.1;
	if (pitchErr > 50) 
	{
		LV = RV = 0;
	}
	//Serial.print(pitch[0]);
	//Serial.print('\t');
/*
	Serial3.print(pitchOffset);
	Serial3.print('\t');
	Serial3.print(yaw[0]);
	Serial3.print('\t');
	Serial3.println(pitch[1]);
	*//*
	Serial.print('\t');
	Serial.println(lPWM);
	*/
	//yawcerr = constrain(yawcerr + yawErr * yKiss, -7, 7);
	//int temp = constrain(yawErr * yKpss + yaw[1] * yKdss + yawcerr, -10, 10);
	//lPWM += temp;
	//rPWM -= temp;
	//lPWM = ((abs(pitchErr)>2) ? 0 : lPWM);
	//rPWM = ((abs(pitchErr)>2) ? 0 : rPWM);
}

void turnByX(float x) {}
void moveByD(int D) {}
void moveByAndTurn(int d, float x) {}
void maintainAndRotate() {}

/*
0 - Calculate Offset.
1 - Maintain Balance and Position
2 - Turn by x degrees
3 - Move x distance
4 - Turn while moving
default - Maintain and Rotate
*/

void setMode(int m) {
	mode = m;
	switch (mode)
	{
	case 0:
		yawOffset = yaw[0];
		break;
	case 1:
		yawOffset = yaw[0];
		yawcerr = 0;
		break;
	case 2: yawOffset = yaw[0];
		break;
	case 3: yawOffset = yaw[0];
		break;
	case 4: yawOffset = yaw[0];
		break;
	default:
		break;
	}
}


void loop() {
	if (!dmpReady) return;

	while (!mpuInterrupt && fifoCount < packetSize) {
	}

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
		I2Cdev::writeBit(0x68, 0x6A, 2, 1);//reset FIFO

	else

		if (mpuIntStatus & 0x02) {

			while (fifoCount < packetSize)
				fifoCount = mpu.getFIFOCount();
			I2Cdev::readBytes(0x68, 0x74, packetSize, fifoBuffer);//read FIFO
			fifoCount -= packetSize;
			readSensors();
			if (Serial3.available()>2) {
				switch (Serial3.read())
				{
				case 'p':pKpss = Serial3.parseFloat();break;
				case 'i':pKdss = Serial3.parseFloat();break;
				case 'd':yKpss = Serial3.parseFloat();break;
				case 'a':yKdss = Serial3.parseFloat();break;
				case 'b':dKdss = Serial3.parseFloat();break;
				case 'c':dKpss = Serial3.parseFloat();break;
				case 's':pitchOffset = Serial3.parseFloat();break;
				case 'e':pKiss = Serial3.parseFloat();break;
				case 't':cPErr = Serial3.parseFloat();break;
				case 'f':yawOffset = Serial3.parseFloat();break;
			}
				Serial3.read();
			}
			
			switch (mode) {
			case 0:
				calculateOffset();
				break;
			case 1:
				stayStill();
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			default:
				maintainAndRotate();
			}
			
			motorControl();
			
		}
}
int16_t mx, my, mz;uint8_t buffer[6];

void readSensors() {
	I2Cdev::readBytes(HMC5883L, 0x03, 6, buffer);
	I2Cdev::writeByte(HMC5883L, 0x02, 0x01);
	
	mx = (((int16_t)buffer[0]) << 8) | buffer[1];
    my = (((int16_t)buffer[4]) << 8) | buffer[5];
    mz = (((int16_t)buffer[2]) << 8) | buffer[3];
	
	float w = ((fifoBuffer[0] << 8) | fifoBuffer[1]) / 16384.0;
	float x = ((fifoBuffer[4] << 8) | fifoBuffer[5]) / 16384.0;
	float y = ((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0;
	float z = ((fifoBuffer[12] << 8) | fifoBuffer[13]) / 16384.0;
	float ty = 2 * (w * x + y * z);
	float tz = w * w - x * x - y * y + z * z;
	//mz = -(((int16_t)buffer[5]) << 8) | buffer[4];
	float tempYaw;
	double dt = (micros() - timer) / 1000000.0;
	timer = micros();
	double Bfy = - my;
	

	tempYaw *= -1;
	pitch[0] = atan((2 * (x * z - w * y)) / sqrt(ty * ty + tz * tz));

	yaw[1] = ((fifoBuffer[24] << 8) | fifoBuffer[25]) / 131.0;
	pitch[1] = ((fifoBuffer[20] << 8) | fifoBuffer[21]) / 131.0;
	double Bfx = mx + mz * sin(pitch[0]);
	tempYaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
	pitch[0] *= RAD_TO_DEG;
	//Serial3.println(tempYaw);
	
	if ((tempYaw < -90 && yaw[0] > 90) || (tempYaw > 90 && yaw[0] < -90)) {
		filter.setAngle(tempYaw);
		yaw[0] = tempYaw;
	}
	else
		yaw[0] = filter.getAngle(tempYaw, yaw[1], dt);

	//yaw[0] = tempYaw;
	
	/*pKpss = analogRead(A0) / 100.0 + 10;
	pKdss = analogRead(A1) / 100.0;
	dKpss = analogRead(A2) / 100.0;
	dKdss = analogRead(A3) / 100.0;
	*/
	/*Serial.print(vL);
	Serial.print('\t');
	Serial.println(vR);*/
}
uint16_t lFrq, rFrq;
double t;
int preScaler[5] = { 1,8,64,256,1024 };

int cPreScale(int freq) 
{
	int x = 0; double err = 10000;int ocr;int tFreq = 0;
	for (int i = 0;i<5;i++)
	{
		ocr = round(F_CPU / (2.0 * preScaler[i] * freq)) - 1;

		if (ocr>65535) { continue; }
		tFreq = F_CPU / (2.0 * preScaler[i] * (ocr + 1));
		
		if (err>abs(tFreq - freq))
		{
			
			err = abs(tFreq - freq);
			x = i;
		}
		
	}
	
	return x;
}
uint8_t pS[] = {B00000001,B00000010,B00000011,B00000100,B00000101};
void motorControl() {
	if (RV > 0)
		PORTG &= ~_BV(5);
	else
		PORTG |= _BV(5);
	if (LV > 0)
		PORTE |= _BV(5);
	else
		PORTE &= ~_BV(5);
	//Serial.println(RV);
	RV = constrain(abs(RV) * 900 / PI, 0, 8000);
	LV = constrain(abs(LV) * 900 / PI, 0, 8000);

	int t;
	TCCR4B = (TCCR4B & 0b11111000) | pS[t = cPreScale(LV)];

	TCNT4 = 0;
	OCR4A = F_CPU / LV / 2 /preScaler[t] - 1;

	TCCR3B = (TCCR3B & 0b11111000) | pS[t = cPreScale(RV)];
	TCNT3 = 0;
	OCR3A = F_CPU / RV / 2 / preScaler[t] - 1;
	//Serial.println(OCR3A);

	/*
	PORTG &= ((3 << 6) | (rDir << 5) | (15));
	PORTE &= ((3 << 6) | (lDir << 5) | (15));
	*/
	

}