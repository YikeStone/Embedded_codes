#define MtAchA 2
#define MtAchB 3
#define MtBchA 11
#define MtBchB 12
#define MtAI1 4
#define MtAI2 7
#define MtBI1 8
#define MtBI2 9
#define MtAE 5
#define MtBE 6

volatile int lastEncA = 0;
volatile long encValA = 0;
volatile int lastEncB = 0;
volatile long encValB = 0;

long lastencValA = 0;
long lastencValB = 0;

int lastMSBA = 0;
int lastLSBA = 0;
int lastMSBB = 0;
int lastLSBB = 0;

void setup() {
	Serial.begin(57600);

	pinMode(MtAchA, INPUT);
	pinMode(MtAchB, INPUT);		//interrupt pins
	pinMode(MtBchA, INPUT);
	pinMode(MtBchB, INPUT);
	
	pinMode(MtAE, OUTPUT);		//pwn pins
	pinMode(MtBE, OUTPUT);

	pinMode(MtAI1, OUTPUT);		//direction pins
	pinMode(MtAI2, OUTPUT);
	pinMode(MtBI1, OUTPUT);
	pinMode(MtBI2, OUTPUT);

	PORTB = (PORTB & 0b11111100) | 0b00000011;
	PORTD = (PORTD & 0b01101111) | 0b10010000;
	digitalWrite(MtAchA, HIGH);
	digitalWrite(MtAchB, HIGH);
	digitalWrite(MtBchA, HIGH);
	digitalWrite(MtBchB, HIGH);

	attachInterrupt(0, updateEncA, CHANGE);
	attachInterrupt(1, updateEncA, CHANGE);

	*digitalPinToPCMSK(MtBchA) |= bit(digitalPinToPCMSKbit(MtBchA));  // enable pin
	PCIFR |= bit(digitalPinToPCICRbit(MtBchA)); // clear any outstanding interrupt
	PCICR |= bit(digitalPinToPCICRbit(MtBchA)); // enable interrupt for the group
	*digitalPinToPCMSK(MtBchB) |= bit(digitalPinToPCMSKbit(MtBchB));  // enable pin
	PCIFR |= bit(digitalPinToPCICRbit(MtBchB)); // clear any outstanding interrupt
	PCICR |= bit(digitalPinToPCICRbit(MtBchB)); // enable interrupt for the group

	analogWrite(MtAE, 1);
	analogWrite(MtBE, 1);
	OCR0A = OCR0B = 0;
}


int pos = 0;
int ang = 0;
int vel = 0;
float pP = 0, iP = 0, dP = 0, alphaP = 0.7;
float pA = 0, iA = 0, dA = 0, alphaA = 0.7;
int errA = 0, perrA = 0, cerrA = 0;
int errP = 0, perrP = 0, cerrP = 0;
int pwmA = 0, pwmB = 0, pwm = 0, outA = 0;


void loop() {

	if (Serial.available() > 2)
	{
		char ch = Serial.read();
		if (ch == 's')
		{
			switch (Serial.read())
			{
			case 'p':
				pos = Serial.parseInt();
				break;
			case 'a':
				ang = Serial.parseInt();
				break;
			case 'v':
				vel = Serial.parseInt();
				if (abs(vel) > 225)
					vel = 255;
				else
					vel = abs(vel);
				break;


				// tuning

			case 'P':
				switch (Serial.read())
				{
				case 'p':
					pP = Serial.parseFloat();
					break;
				case 'i':
					iP = Serial.parseFloat();
					break;
				case 'd':
					dP = Serial.parseFloat();
					break;
				}
			case 'A':
				switch (Serial.read())
				{
				case 'p':
					pA = Serial.parseFloat();
					break;
				case 'i':
					iA = Serial.parseFloat();
					break;
				case 'd':
					dA = Serial.parseFloat();
					break;
				}
			/*
			case '0':
				ang = 0;
				cerrA = 0;
				pos = 0;
				cerrP = 0;
				encValA = encValB = 0;
				break;
				*/
			}
			Serial.read();
		}
	}
	
	errP = 2 * pos - (encValA + encValB);
	cerrP += errP * iP;

	if(vel > 0)
		cerrP = constrain(cerrP, -vel*0.5, vel*0.5);
	else if(vel == 0)
		cerrP = 0;

	pwm = errP * pP + (-perrP * alphaP + errP * (1 - alphaP)) * dP + cerrP;
	
	perrP = errP;
	
	if (vel > 0)
		pwm = constrain(pwm, -vel, vel);
	else if (vel == 0)
		pwm = 0;

	errA = ang - (encValB - encValA);
	cerrA += errA * iA;

	if(vel > 0)
		cerrA = constrain(cerrA, -vel, vel);
	else if(vel == 0)
		cerrA = 0;
	
	outA = errA * pA + (-perrA * alphaA + errA * (1 - alphaA)) * dA + cerrA;

	perrA = errA;
	if (vel > 0)
	{
		outA = constrain(outA, -vel * 0.95, vel * 0.95);
		pwmA = pwm - outA;
		pwmB = pwm + outA;
		pwmA = constrain(pwmA, -255, 255);
		pwmB = constrain(pwmB, -255, 255);
	}
	else if(vel == 0)
	{
		pwmA = pwmB = 0;
		outA = 0;
		errP = 0;
	}
	

	Serial.print(errP);
	Serial.print('\t');
	Serial.print(pwm);
	Serial.print('\t');
	Serial.print(errA);
	Serial.print('\t');
	Serial.print(outA);
	Serial.print('\t');
	Serial.print(pwmA);
	Serial.print('\t');
	Serial.println(pwmB);

	/*
	if (errP == 0)
	{
		cerrP = 0;
		encValA = encValB = pos;
	}

	if (errA == 0)
	{
		ang = 0;
		cerrA = 0;

	}
	*/
	if ((errP == 0) && (errA == 0)) 
	{
		ang = 0;
		cerrA = 0;
		pos = 0;
		cerrP = 0;
		encValA = encValB = 0;
	}

	if (pwmB > 0)
		PORTB = (PORTB & 0b11111100) | 0b00000010;
	else if (pwmB < 0)
	{
		pwmB *= -1;
		PORTB = (PORTB & 0b11111100) | 0b00000001;
	}
	else
		PORTB = PORTB | 0b00000011;

	if (pwmA > 0)
		PORTD = (PORTD & 0b01101111) | 0b00010000;
	else if (pwmA < 0)
	{
		pwmA *= -1;
		PORTD = (PORTD & 0b01101111) | 0b10000000;
	}
	else
		PORTD = PORTD | 0b10010000;
	
	OCR0A = pwmB;
	OCR0B = pwmA;
}

void updateEncA() {
	int MSB = (PIND >> 2) & 0b00000001; //MSB = most significant bit
	int LSB = (PIND >> 3) & 0b00000001; //LSB = least significant bit

	int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
	int sum = (lastEncA << 2) | encoded; //adding it to the previous encoded value

	if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encValA++;
	if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encValA--;

	lastEncA = encoded; //store this value for next time
}

ISR(PCINT0_vect) {
	int MSB = (PINB >> 3) & 0b00000001; //MSB = most significant bit
	int LSB = (PINB >> 4) & 0b00000001; //LSB = least significant bit

	int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
	int sum = (lastEncB << 2) | encoded; //adding it to the previous encoded value

	if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encValB--;
	if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encValB++;

	lastEncB = encoded; //store this value for next time
}