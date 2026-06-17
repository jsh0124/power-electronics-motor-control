#include <stdio.h>
#include <math.h>

double Ts = 100.e-6; //제어주기
double dtCNT = 0.;
double V_ref = 0.;   //지령 전압



__declspec(dllexport) void simuser(double t, double dt, double* in, double* out)   // 실시간으로 계산하는 코드
{

    dtCNT += dt;



    if (dtCNT >= Ts)
    {
        V_ref = 70.;


        dtCNT -= Ts;
    }

    out[0] = V_ref;
    out[1] = 0;
    out[2] = 0;
}