#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Kalman.h"
MPU6050 mpu;
#define mag 0x0C
#define LTM_DIR 7
#define RTM_DIR 4
#define LTM_PWM 6
#define RTM_PWM 5
#define LTM_INT 12
#define RTM_INT 3
#define F 1
#define MPU_INT 2
#define pitch 0
#define yaw 1
#define L 2
#define R 3
#define N 10
#define BACKLASH_COMP 20
float MPU_ZERO = 4.0;
#define ma 1000
#define m 5
#define n 5
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Kalman filter;
uint32_t timer;
int32_t RV;
int32_t LV;
volatile bool rDir = 0, lDir = 0;
float vel_P = -1.1, vel_I = -0.2, cerr_D; float head_P = 0.05, head_I = 0.1, cerr_Yaw;
float Offset[4] = { 0,0,0,0 };
float Offset_dot[4] = { 0,0,0,0 };
float in[4];
float in_dot[4];
float E[4];
float Er[4];/*
			0 Pitch
			1 Yaw
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

volatile bool mpuInterrupt = false;
volatile uint8_t vel_tim = 0;
void dmpDataReady() {
	mpuInterrupt = true;
	vel_tim++;
}
volatile int16_t ERC, ELC; volatile int8_t addR = 0, addL = 0;
void encoderInt() {
	ERC += addR;
}

ISR(PCINT0_vect) {
	ELC += addL;
}

void turnByX(float x) {}
void moveByD(int D) {}
void moveByAndTurn(int d, float x) {}
void maintainAndRotate() {}
void calculateOffset() {}
void setMode(int a) {
	mode = a;
	/*
	0 - Calculate Offset.
	1 - Maintain Balance and Position
	2 - Turn by x degrees
	3 - Move x distance
	4 - Turn while moving
	default - Maintain and Rotate
	*/
	Offset[yaw] = in[yaw];
	switch (mode)
	{
	case 0:
		break;
	case 1:
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

int pos_e[2], pos_er[2];
int x_[] = { -10, -5, 0, 5, 10 },
x_dot[] = { -60, -20, 0, 20, 60 },
err_dom[n], err_dot_dom[m],
out[m][n] = { { -255, -248, -104,    40,   200 },
{ -255, -192,  -48,    96,   255 },
{ -255, -144,    0,   144,   255 },
{ -255,  -96,   48,   192,   255 },
{ -200,  -40,  104,   248,   255 } }
;

void calc_dom(float err, int *x, int *dom, int l, int *pos, int maxx) {

	if (err <= x[0]) {
		dom[0] = maxx;
		pos[0] = pos[1] = 0;
		return;
	}

	if (err >= x[l - 1]) {
		dom[l - 1] = maxx;
		pos[0] = pos[1] = l - 1;
		return;
	}

	for (int i = 0; i < l - 1; i++) {
		if ((err > x[i]) && (err < x[i + 1])) {
			float s = (maxx * 1.0f) / (x[i + 1] - x[i]);
			dom[i] = (int)(-s * (err - x[i + 1]));
			dom[i + 1] = (int)(s * (err - x[i]));
			pos[0] = i;
			pos[1] = i + 1;
			return;
		}
		else
			if (err == x[i]) {
				dom[i] = maxx;
				pos[0] = pos[1] = i;
				return;
			}
	}
}

int calc_fuzzy(float err, int err_dot)
{
	double output = 0;
	calc_dom(err, x_, err_dom, n, pos_e, ma);
	calc_dom(err_dot, x_dot, err_dot_dom, m, pos_er, ma);

	int sum = 0; double t = 0;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++) {
			output += (t = min(err_dot_dom[pos_er[j]], err_dom[pos_e[i]])) * out[pos_er[j]][pos_e[i]];
			sum += t;
		}
	output /= sum;
	return output;
}

void setup() {
	Serial.begin(57600);
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	mpu.initialize();
	*digitalPinToPCMSK(LTM_INT) |= bit(digitalPinToPCMSKbit(LTM_INT));
	PCIFR |= bit(digitalPinToPCICRbit(LTM_INT));
	PCICR |= bit(digitalPinToPCICRbit(LTM_INT));
	pinMode(MPU_INT, INPUT);
	pinMode(LTM_INT, INPUT);
	pinMode(RTM_INT, INPUT);
	pinMode(LTM_DIR, OUTPUT);
	pinMode(RTM_DIR, OUTPUT);
	pinMode(LTM_PWM, OUTPUT);
	pinMode(RTM_PWM, OUTPUT);
	analogWrite(RTM_PWM, 1);
	analogWrite(LTM_PWM, 1);
	mpu.testConnection();
	devStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(-1);
	mpu.setYGyroOffset(-1);
	mpu.setZGyroOffset(255);
	mpu.setXAccelOffset(705);
	mpu.setYAccelOffset(-139);
	mpu.setZAccelOffset(1219);
	mpu.setDMPEnabled(true);
	attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
	attachInterrupt(digitalPinToInterrupt(RTM_INT), encoderInt, CHANGE);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();
	mpu.setI2CBypassEnabled(true);
	I2Cdev::writeByte(mag, 0x0A, 0x1);
	setMode(1);

}

int c = 0;
float p,d,a,b;

void loop()
{
	if (!dmpReady) return;

	while (!mpuInterrupt && fifoCount < packetSize);
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
			get_input();
			if (Serial.available()>2) {
				switch (Serial.read())
				{
					//case 'p':kp = Serial1.parseFloat();break;
				case 'i': MPU_ZERO = Serial.parseFloat();break;
				case 'd': head_I = Serial.parseFloat(); break;
				case 'a': vel_P = Serial.parseFloat(); break;
				case 'b': vel_I = Serial.parseFloat(); break;
				case 'c': head_P = Serial.parseFloat(); break;
				case 's': Offset[L] = Offset[R] = Serial.parseFloat(); break;
				case 'e': c = Serial.parseInt(); break;
				case 'q': p = Serial.parseFloat();break;
				case 'w': d = Serial.parseFloat();break;
				}
				Serial.read();
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

			if (abs(in[pitch]) > 30)
				output(0, 0);

			else {
				LV = constrain(LV, -255, 255);
				RV = constrain(RV, -255, 255);
				output(LV, RV);
			}

		}
}

uint8_t buffer[6];
void get_input() {
	I2Cdev::readBytes(mag, 0x03, 4, buffer);
	I2Cdev::writeByte(mag, 0x0A, 0x1);
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
	float my = (((int16_t)buffer[1]) << 8) | buffer[0];
	float mx = (((int16_t)buffer[3]) << 8) | buffer[2];
	float mz = -(((int16_t)buffer[5]) << 8) | buffer[4];
	in[pitch] = atan((2 * (x * z - w * y)) / sqrt(ty * ty + tz * tz));
	tempYaw = atan2(my, mx + mz * sin(in[pitch])) * 180.0 / 3.14159265;

	in[pitch] = in[pitch] * RAD_TO_DEG - MPU_ZERO;

	in_dot[yaw] = -((fifoBuffer[24] << 8) | fifoBuffer[25]);

	in_dot[pitch] = -((fifoBuffer[20] << 8) | fifoBuffer[21]);

	if ((tempYaw < -90 && in[yaw] > 90) || (tempYaw > 90 && in[yaw] < -90)) {
		filter.setAngle(tempYaw);
		in[yaw] = tempYaw;
	}
	else
		in[yaw] = filter.getAngle(tempYaw, in_dot[yaw] / 131.0, dt);
	//Serial.println(dt);
	if (vel_tim == N) {
		int tempR = ERC, tempL = ELC;
		ERC = ELC = 0;
		in[R] += tempR;
		in_dot[R] = tempR;
		in[L] += tempL;
		in_dot[L] = tempL;
		//Serial.println(in[L] + in[R]);
		vel_tim = 0;
	}
}

void output(int16_t LT, int16_t RT) {

	if (RT != 0) {
		if (RT > 0) {
			if (rDir != F) {
				addR = 1;
				rDir = F;
				PORTD |= _BV(RTM_DIR);
			}
		}
		else
			if (rDir != (!F)) {
				addR = -1;
				rDir = !F;
				PORTD &= ~_BV(RTM_DIR);
			}
		OCR0B = map(abs(RT), 0, 255, BACKLASH_COMP, 200) * 1.2 + 10;
	}
	else
		OCR0B = 0;
	if (LT != 0) {
		if (LT > 0) {
			if (lDir != F) {
				addL = 1;
				lDir = F;
				PORTD |= _BV(LTM_DIR);
			}
		}
		else
			if (lDir != (!F)) {
				addL = -1;
				lDir = !F;
				PORTD &= ~_BV(LTM_DIR);
			}
		OCR0A = map(abs(LT), 0, 255, BACKLASH_COMP, 200) + 3;
	}
	else
		OCR0A = 0;
}
void cal_Angle_Err() {
	for (int i = 0; i < 2; i++) {
		E[i] = Offset[i] - in[i];
		Er[i] = Offset_dot[i] - in_dot[i];
	}
}
void cal_Vel_Err() {
	for (int i = 2; i < 4; i++) {
		E[i] = Offset[i] - in[i];
		Er[i] = Offset_dot[i] - in_dot[i];
	}
}
int head_PI() {
	cerr_Yaw += E[yaw];
	cerr_Yaw = constrain(cerr_Yaw, -50, 50);
	return constrain((head_P * E[yaw] + head_I * cerr_Yaw), -60, 60);
}

float perr;

float vel_PI(float err, float *cerr) {
	err = perr * 0.7 + err * 0.3;
	perr = err;
	//Serial.println(err);
	*cerr += err;
	*cerr = constrain(*cerr, -3, 3);
	return constrain(vel_P * err + vel_I * (*cerr), -8, 8);
}

float dis_PI(int err) {


}

void stayStill() {

	if (vel_tim == 0) {
		cal_Vel_Err();
		Offset[pitch] = (vel_PI(Er[L] + Er[R] + c * (E[L] + E[R]) * 0.3, &cerr_D));
	}
 
	cal_Angle_Err();
  
	LV = RV = calc_fuzzy(E[pitch], Er[pitch]);
	//LV = RV = p * E[pitch] + d * Er[pitch];
	int dif = head_PI();
	LV -= dif;
	RV += dif;
	Serial.print(p);
	Serial.print(' '); Serial.print(d);
  Serial.print(' ');
	Serial.println(E[pitch]);
}
