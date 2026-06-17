#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>

//모터 파라미터
double pole = 4.;
double L_r = 0.060794;
double L_m = 0.059;
double L_s = 0.060794;
double R_s = 0.295;
double R_r = 0.379;
double I_ds_rated = 7.84;
double I_qs_rated = 15.12;
double lambda_dr = 0.4624;
double T_r = 0.060794 / 0.379;
double J = 0.08;
//전류 제어기 변수
double Kp_current = 0.;
double Ki_current = 0.;
double Ka_current = 0.;
double sigma = 0.;
double wcc = 1000.;
double ia_in = 0.;//3상 입력
double ib_in = 0.;
double ic_in = 0.;
double id_in = 0.;//dq 입력
double iq_in = 0.;
double va_out = 0.;
double vb_out = 0.;
double vc_out = 0.;
double d_current_error = 0.;
double d_current_error_int = 0.;
double d_currnet_controller_out = 0.;
double q_current_error = 0.;
double q_current_error_int = 0.;
double q_current_controller_out = 0.;
//속도 제어기 변수
double Kt_speed = 0.;
double Kp_speed = 0.;
double Ki_speed = 0.;
double Ka_speed = 0.;
double wcs = 0.;
double speed = 0.;
double speed_ref = 0.;
double speed_error = 0.;
double speed_error_int = 0.;
double speed_controller_out = 0.;
double speed_controller_out_ref = 0.;
//전체적인 동작
double CNT = 0.;
double Tsamp = 0.0001;
double theta = 0.;
double slip_ref = 0.;
double w_rm = 0.;
double w_e = 0.;
double torque = 0.;

void speed_controller()
{
	wcs = wcc / 20;
	Kt_speed = 1.5 * (pole / 2) * (L_m / L_r) * lambda_dr;
	Kp_speed = J * wcs / Kt_speed;
	Ki_speed = (J * wcs * wcs) / (5 * Kt_speed);
	Ka_speed = 1 / Kp_speed;
	speed_error = speed_ref - speed;
	speed_error_int += (speed_error-Ka_speed*(speed_controller_out- speed_controller_out_ref)) * Tsamp;
	speed_controller_out = (Kp_speed * speed_error) + (Ki_speed * speed_error_int);//q축 전류 지령이 된다.
	if (speed_controller_out > I_qs_rated)
	{
		speed_controller_out_ref = I_qs_rated;
	}
	else speed_controller_out_ref = speed_controller_out;
}//속도 제어기에서 출력으로 전류 지령이 나옴
//여기에서 나온 전류 지령 같은 경우에는 q축 전류 지령임

void d_q_trans()
{
	id_in = 2 * ((cos(theta) * ia_in) + (cos(theta - (2 * M_PI / 3))*ib_in) + (cos(theta + (2 * M_PI / 3))*ic_in)) / 3.;
	iq_in = 2 * ((-sin(theta) * ia_in) - (sin(theta - (2 * M_PI / 3))*ib_in) - (sin(theta + (2 * M_PI / 3))*ic_in)) / 3.;
}//피드백으로 들어온 3상 전류를 동기좌표계 dq전류로 변환

void current_controller()
{
	sigma = 1 - (L_m * L_m / (L_s * L_r));
	Kp_current = sigma * L_s * wcc;
	Ki_current = (R_s + (R_r * (L_m * L_m / (L_r * L_r))))*wcc;
	Ka_current = 1 / Kp_current;
	
	//q축 전류 제어기
	q_current_error = speed_controller_out_ref - iq_in;
	q_current_error_int += q_current_error * Tsamp;
	q_current_controller_out = (Kp_current * q_current_error) + (Ki_current * q_current_error_int) + (w_e * sigma * L_s * id_in);
	
	//d축 전류 제어기
	d_current_error = I_ds_rated - id_in;
	d_current_error_int += d_current_error * Tsamp;
	d_currnet_controller_out = (Kp_current * d_current_error) + (Ki_current * d_current_error_int) - (w_e * sigma * L_s * iq_in);
}//전류 제어기에서 출력으로 전압 지령이 나옴 --> V_d지령과 V_q지령이 나오게 된다 --> 결국에 3상 인버터이기에 3상 abc지령으로 변환 과정 필요
//해당 제어기에서 d,q축의 전류를 제어해 주어야 함
//d, q축 전류 제어기의 출력에서 각 축에 대한 상호 간섭 성분을 제거 해주는 전향제어 과정이 필요함
void abc_trans()
{
	va_out = (cos(theta) * d_currnet_controller_out) - (sin(theta) * q_current_controller_out);
	vb_out = (cos(theta-(2*M_PI/3))* d_currnet_controller_out) - (sin(theta - (2 * M_PI / 3)) * q_current_controller_out);
	vc_out = (cos(theta + (2 * M_PI / 3)) * d_currnet_controller_out) - (sin(theta + (2 * M_PI / 3)) * q_current_controller_out);
}//전류 제어기에서 나온 출력 v_d, v_q를 v_a, v_b, v_c로 변환

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
	CNT += dt;
	speed = in[0];
	ia_in = in[1];
	ib_in = in[2];
	ic_in = in[3];
	w_rm = speed * 4 / 120;
	if (t < 0.5)
	{
		speed_ref = 0.;
	}
	else if (t >= 0.5 && t < 1.5)
	{
		speed_ref = 1000 * t - 500;
	}
	else
	{
		speed_ref = 1000.;
	}
	if (CNT > Tsamp)
	{
		speed_controller();
		slip_ref = speed_controller_out / (I_ds_rated * T_r);
		theta += (slip_ref + (2 * w_rm * M_PI)) * Tsamp;
		if (theta > 2 * M_PI) theta -= 2 * M_PI;
		w_e = slip_ref + (2 * w_rm * M_PI);
		//speed_controller_out_ref = 15.;
		d_q_trans();
		current_controller();
		abc_trans();
		CNT -= Tsamp;
	}
	torque = Kt_speed * speed_controller_out_ref;
	out[0] = va_out/200;
	out[1] = vb_out/200;
	out[2] = vc_out/200;
	out[3] = id_in;
	out[4] = iq_in;
	out[5] = speed_ref;
	out[6] = speed_controller_out_ref;
	out[7] = q_current_controller_out;
	out[8] = d_currnet_controller_out;
	out[9] = theta;
	out[10] = slip_ref;
	out[11] = Kt_speed;
	out[12] = Kp_speed;
	out[13] = Ki_speed;
	out[14] = I_ds_rated;
	out[15] = torque;
}