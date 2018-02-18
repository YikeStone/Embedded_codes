#define LTM_DIR 4
#define RTM_DIR 2
#define LTM_FRQ 5
#define RTM_FRQ 3
#define LF 1
#define RF 0
int BACKLASH_COMP = 0;
int32_t RV = 0;
int32_t LV = 0;
volatile bool rDir = 0, lDir = 0;

IntervalTimer motorR;
IntervalTimer motorL;

volatile int32_t sL = 0; volatile int addL = 0;
volatile bool LPS = 0;
FASTRUN void motorL_callback() {
	if (LPS)
		sL += addL;
	digitalWriteFast(LTM_FRQ, LPS = !LPS);
}

volatile int32_t sR = 0; volatile int addR = 0;
volatile bool RPS = 0;
FASTRUN void motorR_callback() {
	if (RPS)
		sR += addR;
	digitalWriteFast(RTM_FRQ, RPS = !RPS);
}


#define ALT_OUT 0
#if ALT_OUT
bool pRd, pLd;
FASTRUN void output(int32_t LT, int32_t RT) {
	motorL.end();
	motorR.end();

	if (RT != 0) {
		if (RT > 0) {
			if (rDir != RF) {
				addR = 1;
				digitalWriteFast(RTM_DIR, rDir = RF);
				motorR.begin(motorR_callback, 500000.0 / abs(7700));
				motorR.end();
			}
		}
		else {
			if (rDir != (!RF)) {
				addR = -1;
				digitalWriteFast(RTM_DIR, rDir = !RF);
				motorR.begin(motorR_callback, 500000.0 / abs(7700));
				motorR.end();
			}
		}
		motorR.begin(motorR_callback, 500000.0 / abs(RT));
	}



	if (LT != 0) {
		if (LT > 0) {
			if (lDir != LF) {
				addL = 1;
				digitalWriteFast(LTM_DIR, lDir = LF);
				motorL.begin(motorL_callback, 500000.0 / abs(7700));
				motorL.end();
			}
		}
		else {
			if (lDir != (!LF)) {
				addL = -1;
				digitalWriteFast(LTM_DIR, lDir = !LF);
				motorL.begin(motorL_callback, 500000.0 / abs(7700));
				motorL.end();
			}
		}
		motorL.begin(motorL_callback, 500000.0 / abs(LT));
	}
}
#else
FASTRUN void output(int32_t LT, int32_t RT) {

	motorL.end();
	motorR.end();
	sR = sL = 0;
	if (RT != 0) {
		if (RT > 0) {
			if (rDir != RF) {
				addR = 1;
				digitalWriteFast(RTM_DIR, rDir = RF);
			}
			RT += BACKLASH_COMP;
		}
		else {
			if (rDir != (!RF)) {
				addR = -1;
				digitalWriteFast(RTM_DIR, rDir = !RF);
			}
			RT -= BACKLASH_COMP;
		}
		motorR.begin(motorR_callback, 500000.0 / abs(RT));
	}

	if (LT != 0) {
		if (LT > 0) {
			if (lDir != LF) {
				addL = 1;
				digitalWriteFast(LTM_DIR, lDir = LF);
			}
			LT += BACKLASH_COMP;
		}
		else {
			if (lDir != (!LF)) {
				addL = -1;
				digitalWriteFast(LTM_DIR, lDir = !LF);
			}
			LT -= BACKLASH_COMP;
		}
		motorL.begin(motorL_callback, 500000.0 / abs(LT));
	}
}
#endif
