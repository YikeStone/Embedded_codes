#include "SBR_final_teensy.h"


int tem1, c = 0; float p = -0.02, i = -0.0015, d = 0;
float pp = 200.0, dp = 10.0; float tempCEy = 0.0;
float py = 0;
float tempCE = 0.0; float tempE = 0; int diff = 0;

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, 1);
	delay(2000);
	Serial1.begin(115200);
	digitalWrite(13, 0);
	delay(500);
	digitalWrite(13, 1);
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	mpu.initialize();
	
	pinMode(MPU_INT, INPUT);
	pinMode(LTM_DIR, OUTPUT);
	pinMode(RTM_DIR, OUTPUT);
	pinMode(LTM_FRQ, OUTPUT);
	pinMode(RTM_FRQ, OUTPUT);
	
	mpu.testConnection();
	devStatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	mpu.setDMPEnabled(true);
	attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();
	
/*	mpu.setI2CBypassEnabled(true);
	I2Cdev::writeByte(HMC5883L, 0x00, (0x03 << 5) | (0x04 << 2) | 0x00);
	I2Cdev::writeByte(HMC5883L, 0x01, 0x01 << 5);
	I2Cdev::writeByte(HMC5883L, 0x02, 0x01);
*/	setMode(1);

}

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
			if (Serial1.available() > 2) {
				
				switch (Serial1.read())
				{
				//case 'x':x1 = Serial1.parseInt();
				//	Serial1.read();x[x1] = -Serial1.parseInt();x[6 - x1] = -x[x1];
				//	break;
				case 'o': Offset_dot[L] = Offset_dot[R] = Serial1.parseInt();break;
				case 'd':d = Serial1.parseFloat();break;
				case 'p':p = Serial1.parseFloat();break;
				case 'i':i = Serial1.parseFloat();break;
				case 's':MPU_ZERO = Serial1.parseFloat();break;
				case 'y':pp = Serial1.parseFloat();break;
				case 'z':dp = Serial1.parseFloat(); break;
				case 'a':py = Serial1.parseFloat();break;
				case 'b':Offset_dot[L] = Offset_dot[R] = Offset_dot[Y] = 0; tim = 0; break;
				case 't':Offset_dot[Y] = Serial1.parseInt(); Offset_dot[L] = Offset_dot[R] = 0; tim = millis(); break;
				case 'c':BACKLASH_COMP = Serial1.parseInt(); break;
				case 'f':Offset_dot[L] = Offset_dot[R] = Serial1.parseFloat(); Offset_dot[Y] = 0; tim = millis(); break;
				}
				Serial1.read();
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
			
			if (abs(in[P]) > 40)
				output(0, 0);
			else
				output(LV, RV);
		}
}

void display_debug() 
{
	Serial1.print('a');
	Serial1.print(Er[Y]);
	Serial1.print("\t");
	Serial1.print(in[Y]);
	Serial1.print("\t");
	Serial1.print(in[L] - in[R]);
	Serial1.print("\t");
	Serial1.print(tem1);
	Serial1.print("\t");
	Serial1.print(diff);
	Serial1.print("\t");
	Serial1.print(LV-RV);
	Serial1.print("\t");
	Serial1.println(RV);
}


//float f;


FASTRUN void stayStill() {
	cal_Err();
#if PID
	
	if (!vel_tim) {
		tempE = (Er[L] + Er[R]);
		if (tempE == 0) tempCE = 0;
		tempCE += tempE * i;
		tempCE = ((tempCE > 15) ? 15 : (tempCE < -15) ? -15 : tempCE);
		diff = py * Er[Y];
	}
	Offset[P] = tempE * p + tempCE + (in_dot_dot[R] + in_dot_dot[L]) * d;
	if (abs(tempE) > 0)
		Offset[P] = ((Offset[P] > 20) ? 20 : (Offset[P] < -20) ? -20 : Offset[P]);
	else
		Offset[P] = 0;
	cal_Err();
	tem1 = E[P] * pp + Er[P] * dp;

#else
	//Serial1.print(LV);
	//Serial1.print('\t');
	Offset[P] = calc_vel_fuzzy(((E[L] + E[R]) / 2.0), (Er[L] + Er[R]) / 2.0);
	cal_Err();
	tem1 = calc_fuzzy(E[P], Er[P]);
	LV = RV = tem1;
#endif

	LV = tem1 + diff;
	RV = tem1 - diff;
	display_debug();

}
