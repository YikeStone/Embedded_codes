#include "Fuzzy_controller.h"
#include "Motor_driver.h"
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

//#include "Kalman.h"


#define MPU_INT 20
#define PID 1

//#define MPU_ZERO -3
float MPU_ZERO = -5.0;

#define L 0
#define R 1
#define P 2
#define Y 3
#define N 5
//const int16_t magX = -364, magY = 99, magZ = -42;	//HMC5883L Mag Calibration Offsets
volatile uint8_t vel_tim = 0;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

						//Kalman filter;
uint32_t timer;
uint32_t tim;

float Offset[4] = { 0,0,0,0 };
float Offset_dot[4] = { 0,0,0,0 };
float in[4];
float in_dot[4];
float in_dot_dot[2];
float E[4];
float Err[2];
float Er[4];
/*
0 P
1 Y
2 dist_L
3 dist_R
*/
uint8_t mode = 1;/*
				 0 - Calculate Offset.
				 1 - Maintain Balance and Position
				 2 - Turn by x degrees
				 3 - Move x distance
				 4 - Turn while moving
				 default - Maintain and Rotate
				 */
				 //uint8_t buffer[6];int mx, my, mz;

MPU6050 mpu;
//#define HMC5883L 0x1E


volatile bool mpuInterrupt = false;
FASTRUN void dmpDataReady() {
	mpuInterrupt = true;
	vel_tim++;
}

FASTRUN void get_input() {
	/*	I2Cdev::readBytes(HMC5883L, 0x03, 6, buffer);
	I2Cdev::writeByte(HMC5883L, 0x02, 0x01);

	mx = ((int16_t)((buffer[0] << 8) | buffer[1])) - magX;
	my = ((int16_t)((buffer[4] << 8) | buffer[5])) - magY;
	mz = ((int16_t)((buffer[2] << 8) | buffer[3])) - magZ;
	*/

	float w = (int16_t)((fifoBuffer[0] << 8) | fifoBuffer[1]) / 16384.0;
	float x = (int16_t)((fifoBuffer[4] << 8) | fifoBuffer[5]) / 16384.0;
	float y = (int16_t)((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0;
	float z = (int16_t)((fifoBuffer[12] << 8) | fifoBuffer[13]) / 16384.0;
	float ty = 2 * (w * x + y * z);
	float tz = w * w - x * x - y * y + z * z;


	double dt = (micros() - timer) / 1000000.0;
	timer = micros();

	in[P] = atan(2 * (x * z - w * y)) / sqrt(ty * ty + tz * tz);

	//in_dot[Y] = (int16_t)((fifoBuffer[24] << 8) | fifoBuffer[25]);
	in_dot[P] = -(int16_t)((fifoBuffer[20] << 8) | fifoBuffer[21]);

	//	float tempY = atan2(-my, mx + mz * sin(in[P])) * RAD_TO_DEG;

	in[P] = in[P] * RAD_TO_DEG - MPU_ZERO;

	/*	if ((tempY < -90 && in[Y] > 90) || (tempY > 90 && in[Y] < -90)) {
	filter.setAngle(tempY);
	in[Y] = tempY;
	}
	else
	in[Y] = filter.getAngle(tempY, in_dot[Y] / 131.0, dt);
	*/
	if (vel_tim == N) {


		float temp = (sL * 0.2) * DEG_TO_RAD * 6.25;
		in_dot[L] = temp / dt;
		in[L] += temp;


		temp = (sR * 0.2) * DEG_TO_RAD * 6.25;
		in_dot[R] = temp / dt;
		in[R] += temp;


		vel_tim = 0;
		in_dot[Y] = RAD_TO_DEG * (in_dot[L] - in_dot[R]) / 35.0;
		in[Y] = RAD_TO_DEG * (in[L] - in[R]) / 35.0;
		if (in[Y] > 360)
			in[Y] = (int)(in[Y]) % 360;
		else if (in[Y] < -360)
			in[Y] = - ((int)(-in[Y]) % 360);
	}
	/*
	if (vel_tim == N) {
	float temp = (sL * 0.2) * DEG_TO_RAD * 6.25;
	in_dot[L] = (temp - in[L]) / dt;
	in[L] = temp;
	temp = (sR * 0.2) * DEG_TO_RAD * 6.25;
	in_dot[R] = (temp - in[R]) / dt;
	in[R] = temp;
	vel_tim = 0;
	in[Y] = in[L] - in[R];

	in[L] = in_dot[L];
	in[R] = in_dot[R];
	}*/
}



void setMode(int m) {
	mode = m;
	/*
	0 - Calculate Offset.
	1 - Maintain Balance and Position
	2 - Turn by x degrees
	3 - Move x distance
	4 - Turn while moving
	default - Maintain and Rotate
	*/
	Offset[Y] = in[Y];
	switch (mode)
	{
	case 0:
		break;
	case 1:
		Offset[L] = in[L];
		Offset[R] = in[R];
		Offset[P] = 0;
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	default:
		break;
	}
}



void turnByX(float x) {}
void moveByD(int D) {}
void moveByAndTurn(int d, float x) {}
void maintainAndRotate() {}
void calculateOffset() {}

FASTRUN void cal_Err() {
	if ((millis() - tim) >= 2000)
	{
		Offset_dot[L] = Offset_dot[R] = Offset_dot[Y] = 0;
		tim = millis();

		E[L] = Offset[L] - in[L];
		E[R] = Offset[R] - in[R];
		E[Y] = Offset[Y] - in[Y];
		Er[L] = Offset_dot[L] - in_dot[L];
		Er[R] = Offset_dot[R] - in_dot[R];
		Er[Y] = Offset_dot[Y] - in_dot[Y];
	
	}
	if (!vel_tim) 
	{
		E[L] = Offset[L] - in[L];
		E[R] = Offset[R] - in[R];
		E[Y] = Offset[Y] - in[Y];

		Er[L] = Offset_dot[L] - in_dot[L];
		Er[R] = Offset_dot[R] - in_dot[R];
		Er[Y] = Offset_dot[Y] - in_dot[Y];
	}

	E[P] = Offset[P] - in[P];
	Er[P] = Offset_dot[P] - in_dot[P];
}