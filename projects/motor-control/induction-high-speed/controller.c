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
double V_smax = 0.;
double I_smax = 17.03;
//전류 제어기 변수
double Kp_current = 0.;
double Ki_current = 0.;
double Ka_current = 0.;
double sigma = 0.;
double wcc = 0.;
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
double d_flux_current_error = 0.;
double d_flux_current_error_int = 0.;
double d_flux_currnet_controller_out = 0.;
double q_flux_current_error = 0.;
double q_flux_current_error_int = 0.;
double q_flux_current_controller_out = 0.;
double vs_out = 0.;
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
double speed_controller_out_ref = 0.;//q축 전류 지령
//약계자 제어기 변수
double Kt_flux = 0.;
double Kp_flux = 0.;
double Ki_flux = 0.;
double Ka_flux = 0.;
double Kp_flux2 = 0.;
double Ki_flux2 = 0.;
double Ka_flux2 = 0.;
double v_flux_error = 0.;
double v_flux_error_int = 0.;
double flux_weakeng_out = 0.;//이걸로 d축 전류를 만들자
double v_slip_error = 0.;
double v_slip_error_int = 0.;
double flux_weakeng_out2 = 0.;
double I_ds_ref = 0.;
double I_ds_fw = 0.;
double I_ds_fw_slip = 0.;
double I_qs_fw_slip = 0.;
double slip_error = 0.;
double slip_error_int = 0.;
double q_temp = 0.;
//전체적인 동작
double CNT = 0.;
double Tsamp = 0.0001;
double theta = 0.;
double slip_ref = 0.;//w_r
double slip_max = 0.;
double w_rm = 0.;
double w_e = 0.;
double w_base = 374.66;
double w_BT = 2119.52;//1922.87;
double torque = 0.;
double motor_flag = 0.;

void flag_control()
{
	//if (w_e < w_base) 
	//{
	//	motor_flag = 0.;//일정 토크 영역
	//}
	//else if (w_e >= w_base && w_e < w_BT)
	//{
	//	motor_flag = 1.;//일정 출력 영역 : 약계자 영역 1
	//}
	//else if (w_e > w_BT)
	//{
	//	motor_flag = 2.;//특성 영역 : 약계자 영역 2
	//}
	if (w_e > w_BT && slip_max < slip_ref)//&& w_e > w_BT  slip_max < slip_ref
	{
	motor_flag = 2.;
	}
	else if (V_smax < vs_out && w_e >= w_base && w_e < w_BT)//&& w_e >= w_base && w_e < w_BT
	{
		motor_flag = 1.;
	}

	//else motor_flag = 0.;
}

void speed_controller()
{
	wcs = wcc / 20;
	if (motor_flag == 0.) 
	{
		I_ds_ref = I_ds_rated;
		Kt_speed = 1.5 * (pole / 2) * (L_m / L_r) * lambda_dr;
		Kp_speed = J * wcs / Kt_speed;
		Ki_speed = (J * wcs * wcs) / (5 * Kt_speed);
		Ka_speed = 1 / Kp_speed;
		speed_error = speed_ref - speed;
		speed_error_int += (speed_error - Ka_speed * (speed_controller_out - speed_controller_out_ref)) * Tsamp;
		speed_controller_out = (Kp_speed * speed_error) + (Ki_speed * speed_error_int);//q축 전류 지령이 된다.
		if (speed_controller_out > I_qs_rated)
		{
			speed_controller_out_ref = I_qs_rated;
		}
		else speed_controller_out_ref = speed_controller_out;
	}//일정 토크 영역에서의 속도 제어기
}//속도 제어기에서 출력으로 전류 지령이 나옴
//여기에서 나온 전류 지령 같은 경우에는 q축 전류 지령임

void flux_weakening()
{
	if (motor_flag == 1.)
	{
		lambda_dr = L_m * sqrt((((V_smax / w_e) * (V_smax / w_e)) - ((sigma * L_s * I_smax) * (sigma * L_s * I_smax))) / ((L_s * L_s) - (sigma * L_s * L_s)));
		//Kt_flux = 1.;
		I_ds_fw = sqrt((((V_smax / w_e) * (V_smax / w_e)) - ((sigma * L_s * I_smax) * (sigma * L_s * I_smax))) / ((L_s * L_s) - (sigma * L_s * L_s)));
		Kp_flux = 0.0001;
		Ki_flux = 0.07;
		Ka_flux = 1 / Kp_flux;//
		v_flux_error = V_smax - vs_out;//음수가 나와야함 --> 내보내고자 하는 전압이 정격 전압보다 크기에 약계자를 해야하기 때문
		v_flux_error_int += (v_flux_error - Ka_flux * (flux_weakeng_out - I_ds_ref)) * Tsamp;
		flux_weakeng_out = (Kp_flux * v_flux_error) + (Ki_flux * v_flux_error_int) + I_ds_fw;//d축 전류 지령이 된다. + feedforward d축 전류를 더해준다.
		if (flux_weakeng_out > I_ds_rated)
		{
			I_ds_ref = I_ds_rated;
		}
		else if (flux_weakeng_out < -I_ds_rated)
		{
			I_ds_ref = -I_ds_rated;
		}
		else I_ds_ref = flux_weakeng_out;

		speed_controller_out = sqrt((I_smax * I_smax) - (I_ds_ref * I_ds_ref));//이게 문제인거 같음...
		speed_controller_out_ref = speed_controller_out;
		q_temp = speed_controller_out_ref;
	}//일정 출력 영역에서의 약계자 제어기 --> d축 전류 지령을 내보내고 이 전류 지령으로 q축 전류 지령을 만드는게 약계자 제어
	else if (motor_flag == 2.)
	{
		//lambda_dr = L_m * sqrt((((V_smax / w_e) * (V_smax / w_e)) - ((sigma * L_s * I_smax) * (sigma * L_s * I_smax))) / ((L_s * L_s) - (sigma * L_s * L_s)));
		////Kt_flux = 1.;
		//I_ds_fw = sqrt((((V_smax / w_e) * (V_smax / w_e)) - ((sigma * L_s * I_smax) * (sigma * L_s * I_smax))) / ((L_s * L_s) - (sigma * L_s * L_s)));
		//Kp_flux = 0.0001;
		//Ki_flux = 0.07;
		//Ka_flux = 1 / Kp_flux;//
		//v_flux_error = V_smax - vs_out;//음수가 나와야함 --> 내보내고자 하는 전압이 정격 전압보다 크기에 약계자를 해야하기 때문
		//v_flux_error_int += (v_flux_error - Ka_flux * (flux_weakeng_out - I_ds_ref)) * Tsamp;
		//flux_weakeng_out = (Kp_flux * v_flux_error) + (Ki_flux * v_flux_error_int) + I_ds_fw;//d축 전류 지령이 된다. + feedforward d축 전류를 더해준다.
		//if (flux_weakeng_out > I_ds_rated)
		//{
		//	I_ds_ref = I_ds_rated;
		//}
		//else if (flux_weakeng_out < -I_ds_rated)
		//{
		//	I_ds_ref = -I_ds_rated;
		//}
		//else I_ds_ref = flux_weakeng_out;

		//Kp_flux2 = 0.001;
		//Ki_flux2 = 0.07;
		//Ka_flux2 = 1 / Kp_flux2;//
		I_ds_fw_slip = V_smax / (sqrt(2) * w_e * L_s);//d축 전류 에 대한 전향 보상 값
		I_qs_fw_slip = V_smax / (sqrt(2) * w_e * L_s * sigma);
		speed_controller_out_ref = I_qs_fw_slip;
		I_ds_ref = I_ds_fw_slip;
		//v_slip_error = slip_max - slip_ref;//이것도 음수가 나와야함 slip ref가 어떻게 크게 나오는가?
		//v_slip_error_int += (v_slip_error - Ka_flux2 * (flux_weakeng_out2 - speed_controller_out_ref)) * Tsamp;
		//flux_weakeng_out2 = (Kp_flux2 * v_slip_error) + (Ki_flux2 * v_slip_error_int) + q_temp;//d축 전류 지령이 된다. + feedforward d축 전류를 더해준다.
		//if (flux_weakeng_out2 > I_qs_rated)
		//{
		//	speed_controller_out_ref = I_qs_rated;
		//}
		//else if(flux_weakeng_out2 < -I_qs_rated)
		//{
		//	speed_controller_out_ref = -I_qs_rated;
		//}
		//else speed_controller_out_ref = flux_weakeng_out2;
		//q_temp = speed_controller_out_ref;
		//I_ds_ref = speed_controller_out_ref * sigma;
		//speed_controller_out_ref = I_ds_ref / sigma;//q축 전류 지령 --> d축 전류가 줄어들면서 q축 전류도 줄어듦을 의미
	}//슬립이 최대 슬립으로 유지 되는 구간
}

void d_q_trans()
{
	id_in = 2 * ((cos(theta) * ia_in) + (cos(theta - (2 * M_PI / 3)) * ib_in) + (cos(theta + (2 * M_PI / 3)) * ic_in)) / 3.;
	iq_in = 2 * ((-sin(theta) * ia_in) - (sin(theta - (2 * M_PI / 3)) * ib_in) - (sin(theta + (2 * M_PI / 3)) * ic_in)) / 3.;
}//피드백으로 들어온 3상 전류를 동기좌표계 dq전류로 변환

void current_controller()
{
	Kp_current = sigma * L_s * wcc;
	Ki_current = (R_s + (R_r * (L_m * L_m / (L_r * L_r)))) * wcc;
	Ka_current = 1 / Kp_current;

	//q축 전류 제어기
	q_current_error = speed_controller_out_ref - iq_in;
	q_current_error_int += q_current_error * Tsamp;
	q_current_controller_out = (Kp_current * q_current_error) + (Ki_current * q_current_error_int) + (w_e * sigma * L_s * id_in) + (slip_ref * L_m * lambda_dr / L_r);// + (slip_ref * L_m * lambda_dr / L_r)

	//d축 전류 제어기
	d_current_error = I_ds_ref - id_in;
	d_current_error_int += d_current_error * Tsamp;
	d_currnet_controller_out = (Kp_current * d_current_error) + (Ki_current * d_current_error_int) - (w_e * sigma * L_s * iq_in) - (R_r * L_m * lambda_dr / (L_r * L_r));// - (R_r * L_m * lambda_dr / (L_r * L_r))

	//vs_out = sqrt((q_flux_current_controller_out * q_flux_current_controller_out) + (d_flux_currnet_controller_out * d_flux_currnet_controller_out));
	vs_out = sqrt((q_current_controller_out * q_current_controller_out) + (d_currnet_controller_out * d_currnet_controller_out));
}//전류 제어기에서 출력으로 전압 지령이 나옴 --> V_d지령과 V_q지령이 나오게 된다 --> 결국에 3상 인버터이기에 3상 abc지령으로 변환 과정 필요
//해당 제어기에서 d,q축의 전류를 제어해 주어야 함
//d, q축 전류 제어기의 출력에서 각 축에 대한 상호 간섭 성분을 제거 해주는 전향제어 과정이 필요함
void abc_trans()
{
	va_out = (cos(theta) * d_currnet_controller_out) - (sin(theta) * q_current_controller_out);
	vb_out = (cos(theta - (2 * M_PI / 3)) * d_currnet_controller_out) - (sin(theta - (2 * M_PI / 3)) * q_current_controller_out);
	vc_out = (cos(theta + (2 * M_PI / 3)) * d_currnet_controller_out) - (sin(theta + (2 * M_PI / 3)) * q_current_controller_out);
}//전류 제어기에서 나온 출력 v_d, v_q를 v_a, v_b, v_c로 변환

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
	CNT += dt;
	wcc = 2 * M_PI * 1000;
	V_smax = 180;//200 * sqrt(2) / sqrt(3);
	sigma = 1 - (L_m * L_m / (L_s * L_r));
	slip_max = 1. / (T_r * sigma);
	w_BT = sqrt((1 + (sigma * sigma)) / (2 * pow(sigma * L_s, 2))) * (V_smax / I_smax);
	speed = in[0];
	ia_in = in[1];
	ib_in = in[2];
	ic_in = in[3];
	w_rm = speed * 4 / 120;
	if (CNT > Tsamp)
	{
		if (t < 1.0)
		{
			speed_ref = 0.;
		}
		else speed_ref += 0.3;
		flag_control();
		speed_controller();
		flux_weakening();
		slip_ref = speed_controller_out_ref / (I_ds_ref * T_r);//slip_ref = q축 지령/(시정수 X d축 지령)
		theta += (slip_ref + (2 * w_rm * M_PI)) * Tsamp;
		if (theta > 2 * M_PI) theta -= 2 * M_PI;
		w_e = slip_ref + (2 * w_rm * M_PI);
		//lambda_dr = L_m * sqrt((((V_smax / w_e) * (V_smax / w_e)) - ((sigma * L_s * I_smax) * (sigma * L_s * I_smax))) / ((L_s * L_s) - (sigma * L_s * L_s)));
		//speed_controller_out_ref = 15.;
		d_q_trans();
		current_controller();
		abc_trans();
		CNT -= Tsamp;
	}
	torque = Kt_speed * speed_controller_out_ref;
	out[0] = va_out / 200;
	out[1] = vb_out / 200;
	out[2] = vc_out / 200;
	out[3] = id_in;
	out[4] = iq_in;
	out[5] = speed_ref;//속도 지령
	out[6] = speed_controller_out_ref;//q축 지령
	out[7] = q_current_controller_out;//v_q
	out[8] = d_currnet_controller_out;//v_d
	out[9] = theta;
	out[10] = slip_ref;
	out[11] = flux_weakeng_out;
	out[12] = I_ds_fw;
	out[13] = slip_max;
	out[14] = I_ds_ref;
	out[15] = torque;
	out[16] = motor_flag;
	out[17] = vs_out;
	out[18] = w_e;
	out[19] = w_BT;
	out[20] = w_base;
	out[21] = V_smax;
}