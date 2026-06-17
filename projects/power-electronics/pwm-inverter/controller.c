#include <stdio.h>
#include <math.h>

double pi = 3.141592653;
double dtcnt = 0;
double Tsamp = 8.33e-5;
double Io,theta;
double s1 = 0, s2 = 0;
double Io_conj;
double temp1 = 0, temp2 = 0;


__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
{
	dtcnt += dt;
	double Io = 0;//output
	Io = in[0];
	Io_conj = 5 * sin(2 * pi * 10 * t);
	if (Io >= Io_conj + 1)
	{
		s1 = 0;
		s2 = 1;
		temp1 = s1;
		temp2 = s2;
	}
	else if (Io <= Io_conj - 1)
	{
		s1 = 1;
		s2 = 0;
		temp1 = s1;
		temp2 = s2;
	}
	else
	{
		s1 = temp1;
		s2 = temp2;
	}

	out[0] = s1;
	out[1] = s2;
	out[2] = Io_conj;
}