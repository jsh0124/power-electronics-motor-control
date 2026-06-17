#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>


double VDC = 100,
m_s = 0.6,
pi = 3.141592,
dtcnt = 0,
Tsamp = 0.0001;

double angle=0,
theta=0,
theta_n = 0,
theta_m = 0,
theta_rad = 0,
sector = 1,
v_d = 0,
v_q = 0,
v_an_ref = 0,
v_bn_ref = 0,
v_cn_ref = 0,
Duty_a = 0,
Duty_b = 0,
Duty_c = 0,
V_peak = 0,
d_m = 0,
d_n = 0,
d_z = 0,
duty_a=0,
duty_b=0,
duty_c=0
;

double duty_calculate(double theta, double theta_m, double theta_n);

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
	dtcnt += dt;
	if (dtcnt >= Tsamp)
	{
		//theta = fmod(21600*t,360);

		theta += 2 * M_PI * 60*Tsamp;
		if (theta > 2 * M_PI)
		{	
			theta = 0;
		}
		//----------지령부하상전압에 대한 d-q 변환 식-----------
		v_an_ref = 40*cos(2*pi*60*t);
		v_bn_ref = 40 * cos(2 * M_PI * 60 * t - (pi*2)/3.0);
		v_cn_ref = 40 * cos(2 * pi * 60 * t - (pi*4)/3.0);
		v_d = (0.667) * (v_an_ref - (0.5 * v_bn_ref) - (0.5 * v_cn_ref));
		v_q = (0.667) * ((sqrt(3) / 2 * v_bn_ref) - v_cn_ref * sqrt(3) / 2.0);
	
		//------------------------------------------------------
		//각도에 따른 sector 구현 표시
		if (0 <= theta && theta < 60)
		{
			sector = 1;
			theta_m = 0;
			theta_n = M_PI/3.0;
		}
		else if (60 <= theta && theta < 120)
		{
			sector = 2;
			theta_m = M_PI / 3.0;
			theta_n = (M_PI * 2.0) / 3.0;
		}
		else if (120 <= theta && theta < 180)
		{
			sector = 3;
			theta_m = (M_PI * 2.0) / 3.0;
			theta_n = M_PI;
		}
		else if (180 <= theta && theta < 240)
		{
			sector = 4;
			theta_m = M_PI;
			theta_n = (M_PI * 4.0) / 3.0;
		}
		else if (240 <= theta && theta < 300)
		{
			sector = 5;
			theta_m = (M_PI * 4.0) / 3.0;
			theta_n = (M_PI * 5.0) / 3.0;
		}
		else
		{
			sector = 6;
			theta_m = (M_PI * 5.0) / 3.0;
			theta_n = 2.0 * M_PI;
		}
	//

	
		theta_rad = (theta * M_PI) / 180.0;
		double a = sin(M_PI / 3.0);
		d_m = (m_s * sin(theta_n - theta_rad)) / 0.87;
		d_n = (m_s * sin(theta_rad - theta_m)) / 0.87;
		d_z = 1 - (d_m + d_n);
		//스위칭 시간 결정 -- 기준벡터가 얼마나 회전했는지에 따라
		if (sector == 1)
		{
			
			duty_a = (d_m + d_n + (d_z / 2.0));
			duty_b = (d_n + (d_z / 2.0));
			duty_c = (d_z / 2.0);
		}
		else if (sector == 2)
		{
			duty_a = (d_m + (d_z / 2.0));
			duty_b = (d_m + d_n + (d_z / 2.0));
			duty_c = (d_z / 2.0);
		}
		else if (sector == 3)
		{
			duty_a = (d_z / 2.0);
			duty_b = (d_m + d_n + (d_z / 2.0));
			duty_c = (d_n + (d_z / 2.0));
		}
		else if (sector == 4)
		{
			duty_a = (d_z / 2.0);
			duty_b = (d_m + (d_z / 2.0));
			duty_c = (d_m + d_n + (d_z / 2.0));
		}
		else if (sector == 5)
		{
			duty_a = (d_n + (d_z / 2.0));
			duty_b = (d_z / 2.0);
			duty_c = (d_m + d_n + (d_z / 2.0));
		}
		else
		{
			duty_a = (d_m + d_n + (d_z / 2.0));
			duty_b = (d_z / 2.0);
			duty_c = (d_m + (d_z / 2.0));
		}
		dtcnt -= Tsamp;
	}
	out[0] = v_an_ref;
	out[1] = v_bn_ref;
	out[2] = v_cn_ref;
	out[3] = v_d;
	out[4] = v_q;
	out[5] = theta;
	out[6] = sector * 60;
	out[7] = duty_a;
	out[8] = duty_b;
	out[9] = duty_c;
	out[10] = d_m;
	out[11] = d_n;
	out[12] = d_z;

}

double duty_calculate(double theta, double theta_m, double theta_n)
{	
	theta_rad = (theta * M_PI) / 180.0;
	d_m = (m_s * sin(theta_n - theta_rad)) / sin(M_PI/3.0);
	d_n = (m_s * sin(theta_rad - theta_m)) / sin(M_PI/3.0);
	d_z = 1 - (d_m + d_n);
	return d_m,d_n,d_z;
}