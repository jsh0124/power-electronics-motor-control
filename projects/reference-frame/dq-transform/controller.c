#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>

double CNT = 0.;
double Tsamp = 0.0001;
double f_ds = 0.;
double f_ds_inv = 0.;
double f_de = 0.;
double f_qs = 0.;
double f_qs_inv = 0.;
double f_qe = 0.;
double f_a_in = 0.;
double f_b_in = 0.;
double f_c_in = 0.;
double f_a_out = 0.;
double f_b_out = 0.;
double f_c_out = 0.;
double theta = 0.;
double freq = 60.;

void ds_qs_trans()
{
	f_ds = (f_a_in - (f_b_in / 2.) - (f_c_in / 2)) * 2 / 3.;
	f_qs = ((f_b_in * sqrt(3) / 2.) - (f_c_in * sqrt(3) / 2.)) * 2 / 3.;
}

void de_qe_trans()
{
	f_de = cos(theta) * f_ds + sin(theta) * f_qs;
	f_qe = -sin(theta) * f_ds + cos(theta) * f_qs;
}

void ds_qs_inv_trans()
{
	f_ds_inv = cos(theta) * f_de - sin(theta) * f_qe;
	f_qs_inv = sin(theta) * f_de + cos(theta) * f_qe;
}

void abc_inv_trans()
{
	f_a_out = f_ds_inv;
	f_b_out = -(f_ds_inv / 2.) + (f_qs_inv * sqrt(3) / 2.);
	f_c_out = -(f_ds_inv / 2.) - (f_qs_inv * sqrt(3) / 2.);
}

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
	f_a_in = in[0];
	f_b_in = in[1];
	f_c_in = in[2];
	CNT += dt;
	if (CNT > Tsamp)
	{
		theta += 2 * M_PI * freq * Tsamp;
		if (theta > 2 * M_PI) theta -= 2 * M_PI;
		else if (theta < -0) theta += 2 * M_PI;
		ds_qs_trans();
		de_qe_trans();
		ds_qs_inv_trans();
		abc_inv_trans();
		CNT -= Tsamp;
	}
	out[0] = f_ds;
	out[1] = f_qs;
	out[2] = f_de;
	out[3] = f_qe;
	out[4] = f_ds_inv;
	out[5] = f_qs_inv;
	out[6] = f_a_out;
	out[7] = f_b_out;
	out[8] = f_c_out;
	out[9] = theta;
}
