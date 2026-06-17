#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>

double CNT = 0.;
double Tsamp = 0.0001;
double Vdc = 100.;
double theta = 0.;
double MI = 0.9;
double V_d = 0.;
double V_q = 0.;
double v_d_bol = 0.;
double v_q_bol = 0.;
double V_ref = 0.;
double v_d_bol_ref = 0.;
double v_q_bol_ref = 0.;
double V_an_ref = 0.;
double V_bn_ref = 0.;
double V_cn_ref = 0.;
double freq = 60.;
double d_m = 0.;
double d_n = 0.;
double d_z = 0.;
double duty_a = 0.;
double duty_b = 0.;
double duty_c = 0.;
double sector = 0.;
double theta_m = 0.;
double theta_n = 0.;
double theta_bol = 0.;
double theta_dq = 0.;
double var_x = 0.;
double alpha = 0.;

void d_q_trans()
{
	V_d = (2. / 3.) * (V_an_ref - (0.5 * V_bn_ref) - (0.5 * V_cn_ref));
	V_q = (2. / 3.) * ((sqrt(3) * V_bn_ref / 2.) - (sqrt(3) * V_cn_ref / 2.));
	//if (V_d == 0 && V_q > 0)
	//{
	//	theta_dq = M_PI / 2.;
	//}
	//else if (V_d == 0 && V_q < 0)
	//{
	//	theta_dq = -M_PI / 2.;
	//}
	//else
	//{
	//	theta_dq = atan2(V_q, V_d);
	//}
	V_ref = sqrt(V_d * V_d + V_q * V_q);//이걸로 과변조 된지 안된지 판단 ㄱㄱ, 원의 반지름을 의미한다고 생각하면 됨
}

//void abc_trans()
//{
//
//}

void sector_sort()
{
	if (0 <= theta && theta < M_PI/6.)
	{
		sector = 1;
		theta_m = 0;
		theta_n = M_PI / 3.0;
	}
	else if (M_PI / 6. <= theta && theta < M_PI / 3.)
	{
		sector = 2;
		theta_m = 0;
		theta_n = M_PI / 3.0;
	}//0~60도 구간
	else if (M_PI / 3. <= theta && theta < M_PI / 2.)
	{
		sector = 3;
		theta_m = M_PI / 3.0;
		theta_n = (M_PI * 2.0) / 3.0;
	}
	else if (M_PI / 2. <= theta && theta < 2 * M_PI / 3.)
	{
		sector = 4;
		theta_m = M_PI / 3.0;
		theta_n = (M_PI * 2.0) / 3.0;
	}//60~120도 구간
	else if (2 * M_PI / 3. <= theta && theta < 5*M_PI/6)
	{
		sector = 5;
		theta_m = (M_PI * 2.0) / 3.0;
		theta_n = M_PI;
	}
	else if (5 * M_PI / 6 <= theta && theta < M_PI)
	{
		sector = 6;
		theta_m = (M_PI * 2.0) / 3.0;
		theta_n = M_PI;
	}//120~180도 구간
	else if (M_PI <= theta && theta < 7 * M_PI / 6)
	{
		sector = 7;
		theta_m = M_PI;
		theta_n = (M_PI * 4.0) / 3.0;
	}
	else if (7 * M_PI / 6 <= theta && theta < 4 * M_PI / 3.)
	{
		sector = 8;
		theta_m = M_PI;
		theta_n = (M_PI * 4.0) / 3.0;
	}//180~240도 구간
	else if (4 * M_PI / 3. <= theta && theta < 3 * M_PI / 2.)
	{
		sector = 9;
		theta_m = (M_PI * 4.0) / 3.0;
		theta_n = (M_PI * 5.0) / 3.0;
	}
	else if (3 * M_PI / 2. <= theta && theta < 5 * M_PI / 3.)
	{
		sector = 10;
		theta_m = (M_PI * 4.0) / 3.0;
		theta_n = (M_PI * 5.0) / 3.0;
	}//240~300도 구간
	else if(5 * M_PI / 3 <= theta && theta < 11 * M_PI / 6.)
	{
		sector = 11;
		theta_m = (M_PI * 5.0) / 3.0;
		theta_n = 2.0 * M_PI;
	}
	else
	{
		sector = 12;
		theta_m = (M_PI * 5.0) / 3.0;
		theta_n = 2.0 * M_PI;
		}//300~360도 구간
}

void alpha_calc()
{
	var_x = sqrt(V_ref * V_ref - (Vdc * Vdc /3.));// 
	v_d_bol = (Vdc / 2.) + (0.5 * var_x);
	v_q_bol = (Vdc / (2 * sqrt(3))) - (sqrt(3) * var_x / 2.);//비교 하는 값임
	if (v_d_bol == 0 && v_q_bol > 0)
	{
		alpha = M_PI/2.;
	}
	else if (v_d_bol == 0 && v_q_bol < 0)
	{
		alpha = -M_PI/2.;
	}
	else
	{
		alpha = atan2(v_q_bol, v_d_bol);//여기서 나온 각도를 가지고 벡터들의 인가 시간 계산
	}//v_ref로부터 alpha각도가 정해짐
	//정해진 alpha의 각도는 
}

void bolognani_overmodulation()
{
	if (sector == 1 && theta > alpha)
	{
		theta_bol = alpha;
	}
	else if (sector == 2 && theta < (M_PI / 3.) - alpha)
	{
		theta_bol = (M_PI / 3.) - alpha;
	}
	else if (sector == 3 && theta > (M_PI / 3.) + alpha)
	{
		theta_bol = (M_PI / 3.) + alpha;
	}
	else if (sector == 4 && theta < (M_PI * 2. / 3.) - alpha)
	{
		theta_bol = (M_PI * 2. / 3.) - alpha;
	}
	else if (sector == 5 && theta > (M_PI * 2. / 3.) + alpha)
	{
		theta_bol = (M_PI * 2. / 3.) + alpha;
	}
	else if (sector == 6 && theta < M_PI - alpha)
	{
		theta_bol = M_PI - alpha;
	}
	else if (sector == 7 && theta > M_PI + alpha)
	{
		theta_bol = M_PI + alpha;
	}
	else if (sector == 8 && theta < (M_PI * 4. / 3.) - alpha)
	{
		theta_bol = (M_PI * 4. / 3.) - alpha;
	}
	else if (sector == 9 && theta > (M_PI * 4. / 3.) + alpha)
	{
		theta_bol = (M_PI * 4 / 3.) + alpha;
	}
	else if (sector == 10 && theta < (M_PI * 5. / 3.) - alpha)
	{
		theta_bol = (M_PI * 5. / 3.) - alpha;
	}
	else if (sector == 11 && theta > (M_PI * 5. / 3.) + alpha)
	{
		theta_bol = (M_PI * 5. / 3.) + alpha;
	}
	else if (sector == 12 && theta < (M_PI*2) - alpha)
	{
		theta_bol = (M_PI * 2) - alpha;
	}
	else
	{
		theta_bol = theta;
	}
	//v_ref_bol = V_ref * cos(theta_bol);
	v_d_bol_ref = V_ref * cos(theta_bol);
	v_q_bol_ref = V_ref * sin(theta_bol);
	d_m = (MI * sin(theta_n - theta_bol)) / sin(M_PI / 3.);
	d_n = MI * sin(theta_bol - theta_m) / sin(M_PI / 3.);
	d_z = 1 - (d_m + d_n);
}

void duty_calculate()
{
	if (sector == 1 || sector==2)
	{
		duty_a = d_m + d_n + (d_z / 2.);
		duty_b = d_n + (d_z / 2.);
		duty_c = d_z / 2.;
	}
	else if (sector == 3 || sector == 4)
	{
		duty_a = d_m + (d_z / 2.);
		duty_b = d_m + d_n + (d_z / 2.);
		duty_c = (d_z / 2.);
	}
	else if (sector == 5 || sector == 6)
	{
		duty_a = (d_z / 2.);
		duty_b = d_m + d_n + (d_z / 2.);
		duty_c = d_n + (d_z / 2.);
	}
	else if (sector == 7 || sector == 8)
	{
		duty_a = (d_z / 2.);
		duty_b = d_m + (d_z / 2.);
		duty_c = d_m + d_n + (d_z / 2.);
	}
	else if (sector == 9 || sector == 10)
	{
		duty_a = d_n + (d_z / 2.);
		duty_b = (d_z / 2.);
		duty_c = d_m + d_n;
	}
	else
	{
		duty_a = d_m + d_n + (d_z / 2.);
		duty_b = (d_z / 2.);
		duty_c = d_m + (d_z / 2.);
	}
}

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
	CNT += dt;
	V_an_ref = Vdc * 2. / 3. * MI * cos(2 * M_PI * freq * t);
	V_bn_ref = Vdc * 2. / 3. * MI * cos(2 * M_PI * freq * t - (M_PI * 2) / 3.0);
	V_cn_ref = Vdc * 2. / 3. * MI * cos(2 * M_PI * freq * t - (M_PI * 4) / 3.0);
	if (CNT >= Tsamp)
	{
		theta += 2 * M_PI * 60 * Tsamp;//rad단위
		if (theta > 2 * M_PI) theta -= 2 * M_PI;
		else if (theta < 0) theta += 2 * M_PI;
		sector_sort();
		d_q_trans();
		alpha_calc();
		//if (V_ref > Vdc / sqrt(3))
		//{
		//	bolognani_overmodulation();
		//}
		bolognani_overmodulation();
		duty_calculate();
		CNT -= Tsamp;
	}
	out[0] = duty_a;
	out[1] = duty_b;
	out[2] = duty_c;
	out[3] = d_m;
	out[4] = d_n;
	out[5] = d_z;
	out[6] = theta;
	out[7] = theta_bol;//각도로 보기 위함
	out[8] = V_d;
	out[9] = v_d_bol;
	out[10] = V_q;
	out[11] = v_q_bol;
	out[12] = V_ref;
	out[13] = theta_dq * 180 / M_PI;
	out[14] = alpha * 180 / M_PI;
	out[15] = V_an_ref;
	out[16] = V_bn_ref;
	out[17] = V_cn_ref;
	out[18] = sector;
	out[19] = v_d_bol_ref;
	out[20] = v_q_bol_ref;
}
