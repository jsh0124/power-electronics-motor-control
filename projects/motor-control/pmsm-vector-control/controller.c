#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
double CNT = 0.;
double Tsamp = 0.0001;
//모터 파라미터 
double R = 0.201;
double L_d = 0.00489;
double L_q = 0.00577;
double pole = 6.;
double J = 0.08;
double is_rated = 18.7;
//전류 제어기
double d_Kp_current = 0.;
double q_Kp_current = 0.;
double Ki_current = 0.;
double Ka_current = 0.;
double id_err = 0.;
double id_err_int = 0.;
double d_current_controller_out = 0.;
double iq_err = 0.;
double iq_err_int = 0.;
double q_current_controller_out = 0.;
double ia_in = 0.;
double ib_in = 0.;
double ic_in = 0.;
double id_in = 0.;
double iq_in = 0.;
double va_out = 0.;
double vb_out = 0.;
double vc_out = 0.;
double wcc = 0.;
double id_ref = 0.;
double iq_ref = 0.;
double cos_theta = 0.;
double sin_theta = 0.;
double theta = 0.;
double w_rm = 0.;
//속도 제어기
double Kt_speed = 0.;
double Kp_speed = 0.;
double Ki_speed = 0.;
double Ka_speed = 0.;
double speed = 0.;
double speed_ref = 0.;
double speed_err = 0.;
double speed_err_int = 0.;
double speed_controller_out = 0.;
double speed_controller_out_ref = 0.;//고정자 전류 지령 --> 추후 d,q축 전류 지령으로 바꾸어 줘야함
double beta = 0.;
double beta_rad = 0.;
double phi_f = 0.;
double wcs = 0.;

void speed_controller()
{
	wcs = wcc / 20;
	//phi_f = L_d * speed_controller_out;
	phi_f = 0.2823;//영구 자석 상수

	beta = acos((-phi_f + sqrt((phi_f * phi_f) + (8 * (L_d - L_q) * (L_d - L_q) * speed_controller_out_ref * speed_controller_out_ref))) / (4 * (L_d - L_q) * speed_controller_out_ref + 0.000001));
	//beta = 2.;
	//beta_rad = beta * M_PI / 180;
	Kt_speed = (pole / 2.) * (3. / 2.) * ((phi_f * sin(beta)) + ((L_d - L_q) * id_ref * sin(beta)));
	Kp_speed = (J * wcs) / Kt_speed;
	Ki_speed = (J * wcs * wcs) / (5 * Kt_speed);
	Ka_speed = 1 / Kp_speed;
	speed_err = speed_ref - speed;
	speed_err_int += (speed_err - Ka_speed * (speed_controller_out - speed_controller_out_ref)) * Tsamp;
	speed_controller_out = (Kp_speed * speed_err) + (Ki_speed * speed_err_int);
	if (speed_controller_out > is_rated)
	{
		speed_controller_out_ref = is_rated;
	}
	else if (speed_controller_out < -is_rated)
	{
		speed_controller_out_ref = -is_rated;
	}
	else
	{
		speed_controller_out_ref = speed_controller_out;
	}
	id_ref = speed_controller_out_ref * cos(beta);
	iq_ref = speed_controller_out_ref * sin(beta);
}

void current_controller()
{
	d_Kp_current = L_d * wcc;
	q_Kp_current = L_q * wcc;
	Ki_current = R * wcc;
	//q축 전류 제어기
	iq_err = iq_ref - iq_in;
	iq_err_int += iq_err * Tsamp;
	q_current_controller_out = (q_Kp_current * iq_err) + (Ki_current * iq_err_int) + (w_rm * L_d * id_in) + (w_rm * phi_f);
	//d축 전류 제어기
	id_err = id_ref - id_in;
	id_err_int += id_err * Tsamp;
	d_current_controller_out = (d_Kp_current * id_err) + (Ki_current * id_err_int) - (w_rm * L_q * iq_in);
}

void dq_trans()
{
	id_in = (ia_in * cos_theta) + ((ib_in - ic_in) * sin_theta / sqrt(3));
	iq_in = (-ia_in * sin_theta) + ((ib_in - ic_in) * cos_theta / sqrt(3));
}//검증 완료

void abc_trans()
{
	va_out = (cos(theta) * d_current_controller_out) - (sin(theta) * q_current_controller_out);
	vb_out = (cos(theta - (2 * M_PI / 3)) * d_current_controller_out) - (sin(theta - (2 * M_PI / 3)) * q_current_controller_out);
	vc_out = (cos(theta + (2 * M_PI / 3)) * d_current_controller_out) - (sin(theta + (2 * M_PI / 3)) * q_current_controller_out);
}//검증 완료

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
	wcc = 1000 * 2 * M_PI;
	CNT += dt;
	ia_in = in[0];
	ib_in = in[1];
	ic_in = in[2];
	cos_theta = in[3];
	sin_theta = in[4];
	speed = in[5];
	w_rm = speed * 6 / 120 * 2 * M_PI;
	theta = atan2(sin_theta, cos_theta);
	if (theta > M_PI) theta -= 2 * M_PI;
	else if (theta < -M_PI) theta += 2 * M_PI;
	if (t < 1)
	{
		speed_ref = 1000 * t;
	}
	else
	{
		speed_ref = 1000;
	}
	if (CNT > Tsamp)
	{
		dq_trans();
		speed_controller();
		//id_ref = 10.;
		//iq_ref = 20.;
		current_controller();
		//d_current_controller_out = 20.;
		//q_current_controller_out = 20.;
		abc_trans();
		CNT -= Tsamp;
	}
	out[0] = va_out / 200;
	out[1] = vb_out / 200;
	out[2] = vc_out / 200;
	out[3] = id_in;
	out[4] = iq_in;
	out[5] = speed_ref;
	out[6] = speed_controller_out_ref;
	out[7] = q_current_controller_out;
	out[8] = d_current_controller_out;
	out[9] = theta;
	out[10] = id_ref;
	out[11] = iq_ref;
	out[12] = beta;
	out[13] = Kt_speed;
}
