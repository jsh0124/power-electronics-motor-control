#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>

double CNT = 0.0;
double Tsamp = 0.0001;

double speed = 0.0;
double speed_ref = 0.0;
double speed_err = 0.0;
double speed_int = 0.0;
double f_r_out = 0.0;//슬립 주파수
double f_r_ref = 0.0;
double f_m = 0.0;
double f_s_ref = 0.0;
double v_s_ref = 0.0;
double phi = 0.0;
double v_rated = 0.0;
double f_rated = 0.0;
double kp = 0.0;
double ki = 0.0;
double ka = 0.0;
double v_as_ref = 0.0;
double v_bs_ref = 0.0;
double v_cs_ref = 0.0;
double theta = 0.0;
double s_max = 0.2738;//최대 토크가 발생하는 지점에서의 슬립

void speedcontrol();
void voltageref();

__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
    CNT += dt;
    speed = in[0];
    if (t < 2)
    {
        speed_ref = 875. * t;
    }
    else
    {
        speed_ref = 1750;
    }
    v_rated = 220.;
    f_rated = 60.;
    phi = v_rated / (f_rated *sqrt(3));
    if (CNT > Tsamp)
    {
        
        speedcontrol();
        voltageref();
        
        CNT -= Tsamp;
    }
    out[0] = v_as_ref / 250.;
    out[1] = v_bs_ref / 250.;
    out[2] = v_cs_ref / 250.;
    out[3] = speed_ref;
    out[4] = phi;
    out[5] = v_s_ref;
    out[6] = f_s_ref + 0.0000001;
    out[7] = f_r_ref;
    out[8] = kp * speed_err;
    out[9] = ki * speed_int;
    out[10] = theta;
    out[11] = f_r_out;
}

void speedcontrol()
{
    kp = 5;
    ki = 0.01;
    theta += 2 * M_PI * f_s_ref * Tsamp;
    if (theta > 2 * M_PI) theta -= 2 * M_PI;
    else if (theta < 0) theta += 2 * M_PI;
    speed_err = speed_ref - speed;
    speed_int += speed_err* Tsamp;
    f_r_out = (kp * speed_err) + (ki * speed_int);//PI 제어기 설계 + Anti-windup 적용
    if (f_r_out > s_max*60)
    {
        f_r_ref = s_max * 60;
    }
    else if (f_r_out < 0)
    {
        f_r_ref = 0;
    }
    else
    {
        f_r_ref = f_r_out;
    }//슬립주파수 제한기
    //안정한 주행을 위하여, 최대 토크를 내는 슬립 주파수 이하로 제한해야함
    f_m = speed * 4 / 120;

    f_s_ref = f_r_ref + f_m;

    if (f_s_ref < 2)
    {
        v_s_ref = 2 * phi;
        f_s_ref = 2;
        //if (f_s_ref < 0) f_s_ref = 0;
    }//저속 구간에 대한 전압 부스팅!!
    else if (f_s_ref > 60)
    {
        f_s_ref = 60;
        v_s_ref = phi * f_s_ref;
    }//주파수 제한
    else {
        v_s_ref = phi * f_s_ref;
    }

    if (v_s_ref > 220)
    {
        v_s_ref = 220;
    }

}

void voltageref()
{
    v_as_ref = v_s_ref * sin(theta);
    v_bs_ref = v_s_ref * sin(theta - 2 * M_PI / 3);
    v_cs_ref = v_s_ref * sin(theta + 2 * M_PI / 3);
}