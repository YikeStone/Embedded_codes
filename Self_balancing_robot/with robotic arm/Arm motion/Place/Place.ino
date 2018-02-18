void LEDToggle(void);
void SERVOCTRLSPEED(char Speed);
void SERVOSET(unsigned char LSB1, unsigned char MSB1, unsigned char LSB2, unsigned char MSB2, unsigned char LSB3, unsigned char MSB3, unsigned char LSB4, unsigned char MSB4, unsigned char LSB5, unsigned char MSB5, unsigned char LSB6, unsigned char MSB6, unsigned char LSB7, unsigned char MSB7, unsigned char LSB8, unsigned char MSB8, unsigned char LSB9, unsigned char MSB9, unsigned char LSB10, unsigned char MSB10, unsigned char LSB11, unsigned char MSB11, unsigned char LSB12, unsigned char MSB12, unsigned char LSB13, unsigned char MSB13, unsigned char LSB14, unsigned char MSB14, unsigned char LSB15, unsigned char MSB15, unsigned char LSB16, unsigned char MSB16);

#define LED 13
char LEDState = 0;
void setup()
{
  Serial.begin(115200);
  Serial.write("CCCCC");
  delay(1000);
}

void loop()
{
  delay(700);
  SERVOSET(16, 11, 52, 13, 80, 4, 116, 20, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(3440);
  SERVOCTRLSPEED(0);
  delay(600);
  SERVOSET(28, 20, 16, 16, 80, 4, 116, 20, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(646);
  SERVOSET(24, 21, 16, 17, 80, 4, 116, 20, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(517);
  SERVOCTRLSPEED(120);
  delay(200);
  SERVOSET(76, 33, 24, 18, 80, 4, 116, 20, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(3147);
  SERVOSET(76, 33, 24, 18, 80, 4, 92, 5, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(4640);
  SERVOSET(104, 21, 24, 18, 80, 4, 92, 5, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(2713);
  SERVOSET(80, 16, 116, 15, 80, 4, 92, 5, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(1307);
  SERVOSET(16, 11, 52, 13, 80, 4, 92, 5, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16, 100, 16);
  delay(1000);
// Comment or remove the next 5 lines to run code in loop...
  while(1)
  {
    LEDToggle();
    delay(100);
  }
}

void SERVOCTRLSPEED(char Speed)
{
  int count;
  for (count=0; count <16; count++)
  {
    Serial.write(170);
    Serial.write(8);
    Serial.write(0);
    Serial.write(127-Speed);
  }
}

void SERVOSET(unsigned char LSB1, unsigned char MSB1, unsigned char LSB2, unsigned char MSB2, unsigned char LSB3, unsigned char MSB3, unsigned char LSB4, unsigned char MSB4, unsigned char LSB5, unsigned char MSB5, unsigned char LSB6, unsigned char MSB6, unsigned char LSB7, unsigned char MSB7, unsigned char LSB8, unsigned char MSB8, unsigned char LSB9, unsigned char MSB9, unsigned char LSB10, unsigned char MSB10, unsigned char LSB11, unsigned char MSB11, unsigned char LSB12, unsigned char MSB12, unsigned char LSB13, unsigned char MSB13, unsigned char LSB14, unsigned char MSB14, unsigned char LSB15, unsigned char MSB15, unsigned char LSB16, unsigned char MSB16)
{
  Serial.write(170);  Serial.write(10);
  Serial.write(LSB1); Serial.write(MSB1);Serial.write(LSB2);Serial.write(MSB2);Serial.write(LSB3);Serial.write(MSB3);Serial.write(LSB4);Serial.write(MSB4);
  Serial.write(LSB5);Serial.write(MSB5);Serial.write(LSB6);Serial.write(MSB6);Serial.write(LSB7);Serial.write(MSB7);Serial.write(LSB8);Serial.write(MSB8);
  Serial.write(LSB9);Serial.write(MSB9);Serial.write(LSB10);Serial.write(MSB10);Serial.write(LSB11);Serial.write(MSB11);Serial.write(LSB12);Serial.write(MSB12);
  Serial.write(LSB13);Serial.write(MSB13);Serial.write(LSB14);Serial.write(MSB14);Serial.write(LSB15);Serial.write(MSB15);Serial.write(LSB16); Serial.write(MSB16);
  LEDToggle();
}

void LEDToggle(void)
{
  if (LEDState == 0)
    LEDState = 1;
  else
    LEDState = 0;

  digitalWrite(LED, LEDState);
}

