#define ma 1000
int pos_e[2], pos_er[2], pos_s[2], pos_sr[2];
int x[] = { -30, -15, -7, -2, 0, 2, 7, 15, 30 }, n = 9,
x_dot[] = { -70, -30, 0, 30, 70 }, m = 5,
q = 5,
s_dot[] = { -5000,-1500,0,1500,5000 }, o = 5,
err_dom[9], err_dot_dom[5], s_dot_dom[5], s_dom[5],
s[] = { -100,  -5,		  0,    5,  100 },

outs[5][5] = { { 5,	 1,       0,    -1,   -5 },
{ 5,	 1,		  0,    -1,   -5 },
{ 5,	 1,		  0,    -1,   -5 },  //0
{ 5,	 1,		  0,    -1,   -5 },
{ 5,    1,		  0,    -1,   -5 } },
/*	outs[5][5] = { { 35,	30,      30,    20,    0 },
{ 20,	 7,		  7,     0,   -5 },
{ 7,	 2,		  0,    -2,   -7 },  //0
{ 5,	 0,		 -7,    -7,  -20 },
{ 0,  -20,		-30,   -30,  -35 } },*/

out[5][9] =
{ { -7700,	-4500,	-1400,	-800,	 -80,	 200,	800,	3000,   4000 },
{ -6000,	-4000,	-1200,	-700,	 -10,	 350,	900,	3300,   4500 },
{ -5000,	-3500,	-1000, 	-550,      0,	 550,	1000,	3500,   5000 },   //0
{ -4500,	-3300,	-900,	-350,	  10,	 700,	1200,	4000,	6000 },
{ -4000,	-3000,	-800,	-200,	  80,	 800, 	1400,	4500,	7700 } }
;
FASTRUN void calc_dom(float err, int x[], int dom[], int n, int pos[]) {

	if (err <= x[0]) {
		dom[0] = ma;
		pos[0] = pos[1] = 0;
		return;
	}

	if (err >= x[n - 1]) {
		dom[n - 1] = ma;
		pos[0] = pos[1] = n - 1;
		return;
	}

	for (int i = 0; i < n - 1; i++) {
		if ((err > x[i]) && (err < x[i + 1])) {
			float m = (ma * 1.0f) / (x[i + 1] - x[i]);
			dom[i] = (int)(-m * (err - x[i + 1]));
			dom[i + 1] = (int)(m * (err - x[i]));
			pos[0] = i;
			pos[1] = i + 1;
			return;
		}
		else
			if (err == x[i]) {
				dom[i] = ma;
				pos[0] = pos[1] = i;
				return;
			}
	}
}

FASTRUN float calc_vel_fuzzy(float err, int err_dot)
{
	float output = 0;
	calc_dom(err, s, s_dom, q, pos_s);
	calc_dom(err_dot, s_dot, s_dot_dom, o, pos_sr);
	//calc_dom(pre_V, V, V_dom, o, pos_V);
	float sum = 0; float t = 0;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++) {
			output += (t = ((s_dot_dom[pos_sr[j]] < s_dom[pos_s[i]]) ? s_dot_dom[pos_sr[j]] : s_dom[pos_s[i]])) * outs[pos_sr[j]][pos_s[i]];
			sum += t;
		}
	output /= sum;
	return output;
}
FASTRUN int calc_fuzzy(float err, int err_dot)
{
	int output = 0;
	calc_dom(err, x, err_dom, n, pos_e);
	calc_dom(err_dot, x_dot, err_dot_dom, m, pos_er);
	//calc_dom(pre_V, V, V_dom, o, pos_V);
	float sum = 0; int t = 0;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++) {
			output += (t = ((err_dot_dom[pos_er[j]] < err_dom[pos_e[i]]) ? err_dot_dom[pos_er[j]] : err_dom[pos_e[i]])) * out[pos_er[j]][pos_e[i]];
			sum += t;
		}
	output /= sum;
	return output;
}

