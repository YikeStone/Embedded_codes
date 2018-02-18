#define MA1 4
#define MA2 5
#define MB1 7
#define MB2 8
#define EA 6
#define EB 9

//commented out part was for encoders. And it's incomplete. Looking forward to complete it when I get encoder motors.

/*#define ACHA 3
#define ACHB 11
#define BCHA 2
#define BCHB 12
volatile int32_t a = 0;
volatile int32_t b = 0;
*/
void setup()
{
	Serial.begin(57600);
	//attachInterrupt(digitalPinToInterrupt(BCHA), BMI,RISING);
	//attachInterrupt(digitalPinToInterrupt(ACHA), AMI,RISING);
  /* add setup code here */
	pinMode(MA1,OUTPUT);
	pinMode(MA2, OUTPUT);
	pinMode(MB1, OUTPUT);
	pinMode(MB2, OUTPUT);
	pinMode(EA, OUTPUT);
	pinMode(EB, OUTPUT);

}
/*
void BMI() 
{
	a += ((digitalRead(ACHB)) ? 1 : -1);
}

void AMI()
{
	b += ((digitalRead(BCHB)) ? 1 : -1);
}
*/
int r=0, p=0,lp=0,rp=0;
void loop()
{
	if (Serial.available()) 
	{
		Serial.readStringUntil(',');
		Serial.readStringUntil(',');
		Serial.parseFloat();
		Serial.read();
		p = -5 - Serial.parseFloat();
		p = constrain(p, -40, 40);
		Serial.read();
		r = Serial.parseFloat();
		r = constrain(r, -40, 40);
		Serial.read();
		Serial.read();

		rp =  - r * 6.375 - abs(p) * p * 0.16;
		lp =  - r * 6.375 + abs(p) * p * 0.16;
		if (r < 0)
		{
			if (lp > 0)
			{
				digitalWrite(MA1, 0);
				digitalWrite(MA2, 1);
			}
			else
			{
				digitalWrite(MA1, 1);
				digitalWrite(MA2, 0);
			}
			if (rp > 0)
			{
				digitalWrite(MB1, 0);
				digitalWrite(MB2, 1);
			}
			else
			{
				digitalWrite(MB1, 1);
				digitalWrite(MB2, 0);
			}

		}
		else
		{
			if (lp < 0)
			{
				digitalWrite(MA1, 1);
				digitalWrite(MA2, 0);
			}
			else
			{
				digitalWrite(MA1, 0);
				digitalWrite(MA2, 1);
			}
			if (rp < 0)
			{
				digitalWrite(MB1, 1);
				digitalWrite(MB2, 0);
			}
			else
			{
				digitalWrite(MB1, 0);
				digitalWrite(MB2, 1);
			}
		}


		analogWrite(EA, constrain(abs(lp), 80, 255));
		analogWrite(EB, constrain(abs(rp), 35, 255));
	}
}
