#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <I2Cdev.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define LTMDIR 7
#define RTMDIR 4
#define LTMPWM 6
#define RTMPWM 5
#define LTMINT 2
#define RTMINT 3
Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
const uint8_t AK8975 = 0x0C; // Address of magnetometer

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
int16_t tempRaw;

double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556
bool lDir,rDir;
float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];


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
    analogWrite(RTMPWM,( ( abs( a ) <= 100 ) ? a + 25: 0 ));
    analogWrite(LTMPWM,( ( abs( a ) <= 100 ) ? a + 25: 0 ));
}


void setup() {
  delay(100); // Wait for sensors to get ready
    pinMode(LTMINT, INPUT);
    pinMode(RTMINT, INPUT);
    pinMode(LTMDIR, OUTPUT);
    pinMode(RTMDIR, OUTPUT);
    pinMode(LTMPWM, OUTPUT);
    pinMode(RTMPWM, OUTPUT);
    
  Serial.begin(57600); delay(100);
  Wire.begin();Serial.println('h');
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  I2Cdev::writeBits(MPU6050, 0x68, 2, 3, 0x01);
  I2Cdev::writeBit(MPU6050, 0x6B, 6, 0);
  I2Cdev::writeBit(MPU6050, 0x6A, 5, 0);
  I2Cdev::writeBit(MPU6050, 0x37, 1, 1);

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  I2Cdev::writeBytes(MPU6050, 0x19, 4, i2cData); // Write to all four registers at once
  I2Cdev::writeByte(MPU6050, 0x6B, 0x01);// PLL with X axis gyroscope reference and disable sleep mode
   
  I2Cdev::readByte(MPU6050, 0x75, i2cData,1000);
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

//  calibrateMag();

  delay(100); // Wait for sensors to stabilize

  /* Set Kalman and gyro starting angle */
  updateMPU6050();
  updateAK8975();
  updatePitchRoll();
  updateYaw();

  kalmanX.setAngle(roll); // First set roll starting angle
  gyroXangle = roll;
  compAngleX = roll;

  kalmanY.setAngle(pitch); // Then pitch
  gyroYangle = pitch;
  compAngleY = pitch;

  kalmanZ.setAngle(yaw); // And finally yaw
  gyroZangle = yaw;
  compAngleZ = yaw;

  timer = micros(); // Initialize the timer
}
float KP = 0,KI = 0, KD = 0;float cerr = 0;float perr = 0;
float setPitch = 0;
void control(){
  float err = setPitch - kalAngleY;
  cerr+=err;
  cerr=(cerr>10?10:cerr);
  float out = KP * (err) + KI*cerr + KD * (perr - err);
  if(out<0)rDir=lDir=1;
  else rDir=lDir=0;
  motorControl(out,out);
  }


void loop() {
  /* Update all the IMU values */
  updateMPU6050();
  updateAK8975();
  if(Serial.available()>2){
                switch (Serial.read())
                {
                case 'p':KP = Serial.parseFloat();break;
                case 'i':KI = Serial.parseFloat();break;
                case 'd':KD = Serial.parseFloat();break;
                case 's':setPitch = Serial.parseFloat();break;
                //case 'b':kcgi = Serial.parseFloat();break;
                //case 'c':kcgd = Serial.parseFloat();break;*/
                //case 's':initAOffset  = Serial.parseFloat();break;
                //case 'e':mov  = Serial.parseInt();break;
                //case 't':angs = Serial.parseFloat();break;
                //case 'a':an=Serial.parseFloat();break;
                //case 'm':m = Serial.parseFloat();break;
                }
              Serial.read();
              }
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();


  /* Roll and pitch estimation */
  updatePitchRoll();
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif


  /* Yaw estimation */
  updateYaw();
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);
    compAngleZ = yaw;
    kalAngleZ = yaw;
    gyroZangle = yaw;
  } else
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


  /* Estimate angles using gyro only */
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
  //gyroYangle += kalmanY.getRate() * dt;
  //gyroZangle += kalmanZ.getRate() * dt;

  /* Estimate angles using complimentary filter */
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;
control();

  /* Print Data */
#if 1
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

  Serial.print("\t");

  Serial.print(yaw); Serial.print("\t");
  Serial.print(gyroZangle); Serial.print("\t");
  Serial.print(compAngleZ); Serial.print("\t");
  Serial.print(kalAngleZ); Serial.print("\t");
#endif
  Serial.println();

  delay(10);
}

void updateMPU6050() {
  I2Cdev::readBytes(MPU6050, 0x3B, 14, i2cData); // Get accelerometer and gyroscope values
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
}

void updateAK8975() {
  I2Cdev::writeByte(AK8975, 0x0A, 0x1);
  delay(10);
  I2Cdev::readBytes(AK8975, 0x03, 6, i2cData);
  magY = ((i2cData[1] << 8) | i2cData[0]);
  magX = ((i2cData[3] << 8) | i2cData[2]);
  magZ = ((i2cData[5] << 8) | i2cData[4]);
}

void updatePitchRoll() {
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(accX, accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
/*  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;
*/
  double rollAngle = kalAngleX * DEG_TO_RAD;
  double pitchAngle = kalAngleY * DEG_TO_RAD;
  
  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;  
  //yaw *= -1;
}
