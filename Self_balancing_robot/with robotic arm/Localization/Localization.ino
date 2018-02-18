#include <NewPing.h>
#define SONAR_NUM 5
#define FR_R 9
#define FR_L 6
#define RE_L 4
#define LT_F 8
#define LT_R 4
#define FILTER 0

unsigned long pingTimer[SONAR_NUM];
unsigned int cm[SONAR_NUM];
int dur[SONAR_NUM] = {0,0,0,0,0};
float dis[SONAR_NUM] = {0,0,0,0,0};
const int SONAR[SONAR_NUM] = {RE_L, LT_R, FR_L, LT_F, FR_R};


void setup()
{
	Serial.begin(115200);
}
uint32_t t = 0;
void loop() {
	t = millis();
	oneSensorCycle();
	/*for (float i : dis)
	{
		Serial.print(i);
		Serial.print('\t');
	}*/
	Serial.println(millis() - t);
	//delay(100);
}


uint8_t port;
void oneSensorCycle() {
	for (int i = 0;i < SONAR_NUM;i++) {
		if (i < 3) 
		{
			DDRD |= _BV(SONAR[i]);
			PORTD &= ~_BV(SONAR[i]);
		}
		else 
		{
			DDRB |= _BV(SONAR[i]-8);
			PORTB &= ~_BV(SONAR[i]-8);
		}
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		if (i < 3)
			PORTD |= _BV(SONAR[i]);
		else 
			PORTB |= _BV(SONAR[i] - 8);
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");__asm__("nop\n\t");
		if (i < 3)
		{
			PORTD &= ~_BV(SONAR[i]);
			DDRD &= ~_BV(SONAR[i]);
		}
		else
		{
			PORTB &= ~_BV(SONAR[i] - 8);
			DDRB &= ~_BV(SONAR[i] - 8);
		}
#if FILTER
		dur[i] = dur[i] * 0.75 + pulseIn(SONAR[i], 1) * 0.25;
#else
		dur[i] = pulseIn(SONAR[i], 1);
		dis[i] = 330.0 * dur[i] / 1000000.0;
#endif
		delay(1);
	}
}