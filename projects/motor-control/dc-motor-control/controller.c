#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>

//Tsamp=psim time
//전체적인 시뮬레이션 관련 변수
double CNT = 0;
double Tsamp = 0.0001;//10kHz

//모터 파라미터
double R_a = 0.1,
R_f =20,
L_a = 0.02,
L_f = 0.95,
J=0.075,
v_rated = 200,
I_a_rated = 10,
I_f_rated = 2,
n_rated = 1000.;

//전류 제어기 변수
double wcc = 400.;
double Ia_Err = 0.0;
double Ia_ref = 0.0;
double Ia_Errlnt = 0.0;
double Ia_out = 0.0;
double Ia_in = 0.0;
double kp_Ia = 0.0;
double ki_Ia = 0.0;
double ka_Ia = 0.0;
double If_Errlnt = 0.0;
double If_Err = 0.0;
double If_ref = 2.0;
double If_in = 0.0;
double If_out = 0.0;
double kp_If = 0.0;
double ki_If = 0.0;
double ka_If = 0.0;
double v_t_a = 0.0;
double v_t_f = 0.0;
double v_H_a = 0.0;
double v_L_a = 0.0;
double v_H_f = 0.0;
double v_L_f = 0.0;
double Laf = 0.0;
double kpa_out = 0.0;
double kia_out = 0.0;

//속도 제어기 변수
double speed_Err = 0.0;
double speed_Errlnt = 0.0;
double speed_out = 0.0;
double kp_speed = 0.0;
double ki_speed = 0.0;
double speed_ref = 0.0;
double speed_in = 0.0;
double wcs = 0.0;
double kt_speed = 0.0;
double ka_speed = 0.0;

//약계자 제어기 변수
double kp_flux = 0.0;
double ki_flux = 0.0;
double ka_flux = 0.0;
double va_max = 0.0;
double va_flux_error = 0.0;
double va_flux_integ = 0.0;
double flux_out = 0.0;
double v_f_flux_prev = 0.0;
double v_f_flux_out = 0.0;

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
{
	Ia_in = in[0];
	speed_in = in[1];
	If_in = in[2];

	CNT += dt;
	if (t < 1.5)
	{
		speed_ref = (850 / 1.5) * t;
	}
	else if (t >= 1.5 && t < 2)
	{
		speed_ref = 850;
	}
	else if (t >= 2 && t < 3.5)
	{
		speed_ref = (350 / 1.5) * t + (1150 / 3);
	}
	else if (t > 3.5)
	{
		speed_ref = 1200;
	}// 지령 속도에 대한 함수 표현

	if (CNT > Tsamp)
	{
		//if (t < 0.5)
		//{
		//	speed_ref = 0;
		//	Ia_ref = 0;
		//}
		//else
		//{
		//	speed_ref = n_rated;
		//	Ia_ref = I_a_rated;
		//}
		speed_controller();
		armature_current_controller();
		fluxweakening();
		field_current_controller();
		CNT = 0;
	}
	out[0] = v_H_a;
	out[1] = v_L_a;
	out[2] = v_H_f;
	out[3] = v_L_f;
	out[4] = speed_ref;
	out[5] = Ia_ref;
	out[6] = Ia_Err;//kpa_out;
	out[7] = If_ref;
	out[8] = va_flux_error;
	out[9] = v_f_flux_prev;

}

int armature_current_controller()
{
	kp_Ia = wcc * L_a;
	ki_Ia = R_a * wcc;
	ka_Ia = 1. / kp_Ia;
	Laf = kt_speed / I_f_rated;
	Ia_Err = Ia_ref - Ia_in;
	Ia_Errlnt += (Ia_Err - ka_Ia*(Ia_out - v_t_a)) * Tsamp;
	Ia_out = (kp_Ia * Ia_Err) + (ki_Ia * Ia_Errlnt) + Laf*If_in*(speed_in*2*M_PI/60);

	kpa_out = kp_Ia * Ia_Err;//p게인에 의한 영향 확인
	kia_out = ki_Ia * Ia_Errlnt;//i게인에 의한 영향 확인

	if (Ia_out > v_rated)
	{
		v_t_a = v_rated;
	}
	else if (Ia_out < -v_rated)
	{
		v_t_a = -v_rated;
	}
	else
	{
		v_t_a = Ia_out;
	}
	v_H_a = v_t_a;
	v_L_a = -v_t_a;
}

int speed_controller()
{
	wcs = wcc / 5;
	kt_speed = (v_rated - (R_a * I_a_rated)) / ((n_rated * 2 * M_PI) / 60.);
	kp_speed = (J * wcs) / kt_speed;
	ki_speed = (J * wcs * wcs) / (5 * kt_speed);
	ka_speed = 1. / kp_speed;
	speed_Err = speed_ref - speed_in;
	speed_Errlnt += (speed_Err - ka_speed*(speed_out-Ia_ref))* Tsamp;
	speed_out = (kp_speed * speed_Err) + (ki_speed * speed_Errlnt);//speed out은 전류제어기에 들어가는 전류지령의미
	if (speed_out > I_a_rated)
	{
		Ia_ref = I_a_rated;
	}
	else if (speed_out < -I_a_rated)
	{
		Ia_ref = -I_a_rated;
	}
	else
	{
		Ia_ref = speed_out;
	}
}

int fluxweakening()
{
	va_max = 0.95*v_rated;
	kp_flux = 0.01;
	ki_flux = 0.2;
	ka_flux = 1. / kp_flux;

	if(v_t_a>va_max && speed_ref>1000)
	{
		va_flux_error = va_max - Ia_out;
		va_flux_integ += (va_flux_error - ka_flux * (Ia_out - v_t_a)) * Tsamp;
		v_f_flux_out = kp_flux * va_flux_error + ki_flux * va_flux_integ;
		v_f_flux_prev = I_f_rated + v_f_flux_out;
		if (v_f_flux_prev > I_f_rated)
		{
			If_ref = I_f_rated;
		}
		else if (v_f_flux_prev < -v_rated)
		{
			If_ref = -I_f_rated;
		}
		else
		{
			If_ref = v_f_flux_prev;
		}
	}
}

int field_current_controller()
{
	kp_If = wcc * L_f;
	ki_If = R_f * wcc;
	ka_If = 1. / kp_If;
	If_Err = If_ref - If_in;
	If_Errlnt += (If_Err - ka_If * (If_out - v_t_f)) * Tsamp;
	If_out = (kp_If * If_Err) + (ki_If * If_Errlnt);
	if (If_out > v_rated)
	{
		v_t_f = v_rated;
	}
	else if (If_out < -v_rated)
	{
		v_t_f = -v_rated;
	}
	else
	{
		v_t_f = If_out;
	}
	v_H_f = v_t_f;
	v_L_f = -v_t_f;
}