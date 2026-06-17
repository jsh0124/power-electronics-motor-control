#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>


double VDC = 100,
m_s = 0.6,
dtcnt = 0,
Tsamp = 0.0001;

double angle = 0,
theta = 0,
theta_n = 0,
theta_m = 0,
theta_rad = 0,
sector = 1,
v_d = 0,
v_q = 0,
v_an_ref = 0,
v_bn_ref = 0,
v_cn_ref = 0,
v_as_ref = 0,
v_bs_ref = 0,
v_cs_ref = 0,
v_mid_ref = 0,
v_an_nom = 0,
v_bn_nom = 0,
v_cn_nom = 0,
v_sn = 0,
Duty_a = 0,
Duty_b = 0,
Duty_c = 0,
V_peak = 0,
d_m = 0,
d_n = 0,
d_z = 0,
duty_a = 0,
duty_b = 0,
duty_c = 0,
v_max = -40,
v_min = 40
;


__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)
// t는 흘러가는 시간, dt는 피심의 time step 연동해서 들어감, in,out 핀 연동
{
    dtcnt += dt;
    if (dtcnt >= Tsamp)
    {
        //theta = fmod(21600*t,360);
        v_max = -40;
        v_min = 40;
        theta += 2 * M_PI * 60 * Tsamp;
        if (theta > 2 * M_PI)
        {
            theta = 0;
        }
        //MI = 0.8
        v_as_ref = 40 * cos(2 * M_PI * 60 * t);
        v_bs_ref = 40 * cos(2 * M_PI * 60 * t - (M_PI * 2) / 3.0);
        v_cs_ref = 40 * cos(2 * M_PI * 60 * t - (M_PI * 4) / 3.0);
        if (v_as_ref >= v_max)
        {
            v_max = v_as_ref;
            if (v_bs_ref >= v_max)
            {
                v_max = v_bs_ref;
                if (v_cs_ref >= v_max)
                {
                    v_max = v_cs_ref;
                }
            }
            else if (v_cs_ref >= v_max)
            {
                v_max = v_cs_ref;
            }
        }

        if (v_as_ref <= v_min)
        {
            v_min = v_as_ref;
            if (v_bs_ref <= v_min)
            {
                v_min = v_bs_ref;
                if (v_cs_ref <= v_min)
                {
                    v_min = v_cs_ref;
                }
            }
            else if (v_cs_ref <= v_min)
            {
                v_min = v_cs_ref;
            }
        }
        //v_sn = 50 - v_max; //+120 degree DPWM 제어
        v_sn = -50 - v_min;
        v_an_ref = v_as_ref + v_sn;
        v_bn_ref = v_bs_ref + v_sn;
        v_cn_ref = v_cs_ref + v_sn;
        v_an_nom = v_an_ref * 2.0 / 100.0; //V_dc / 2로 정규화 하는 과정
        v_bn_nom = v_bn_ref * 2.0 / 100.0;
        v_cn_nom = v_cn_ref * 2.0 / 100.0;
        //v_d = (0.667) * (v_an_ref - (0.5 * v_bn_ref) - (0.5 * v_cn_ref));
        //v_q = (0.667) * ((sqrt(3) / 2 * v_bn_ref) - v_cn_ref * sqrt(3) / 2.0);

        dtcnt -= Tsamp;
    }
    out[0] = v_as_ref;
    out[1] = v_bs_ref;
    out[2] = v_cs_ref;
    out[3] = v_an_ref;
    out[4] = v_bn_ref;
    out[5] = v_cn_ref;
    out[6] = v_an_nom;
    out[7] = v_bn_nom;
    out[8] = v_cn_nom;
    out[9] = v_max;
    out[10] = v_min;
    out[11] = v_sn;
}
