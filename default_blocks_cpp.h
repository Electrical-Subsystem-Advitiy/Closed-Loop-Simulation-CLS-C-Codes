#include<iostream>
#include<math.h>
#include<time.h>
#include<stdexcept>
#include "satellite_cpp.h"

using namespace std;

float* sunsensor(Satellite sat)
{
    float v_sv_o[3] = sat.getSun_o();
	quat v_q_BO = sat.getQ_BO();
	float* v_sv_b = quatRotate(v_q_BO, v_sv_o);

	return v_sv_b;
}

float* magnetometer(Satellite sat)
{
    float v_B_o[3] = sat.getMag_o();
    quat v_q_BO = sat.getQ_BO();
    float* v_B_b = quatRotate(v_q_BO, v_B_o);

    return v_B_b;
}

float* gps(Satellite sat)
{
    float v_pos_m[3], v_vel_m[3], time_m, res[7];
    for(int i = 0; i<3; i++)
    {
        v_pos_m[i] = (sat.getPos())[i]
        v_vel_m[i] = (sat.getVel())[i]
    }
    time_m = sat.getTime()
    res = {v_pos_m[0], v_pos_m[1], v_pos_m[2], v_vel_m[0], v_vel_m[1], v_vel_m[2], time_m};
    return res;
}

float* gyroscope(Satellite sat)
{
    return (sat.getW_BI_b());
}

float* J2_propagator(Satellite sat)
{
    float v_pos_m[3], v_vel_m[3], time_m, res[7];
    for(int i = 0; i<3; i++)
    {
        v_pos_m[i] = (sat.getPos())[i]
        v_vel_m[i] = (sat.getVel())[i]
    }
    time_m = sat.getTime()
    res = {v_pos_m[0], v_pos_m[1], v_pos_m[2], v_vel_m[0], v_vel_m[1], v_vel_m[2], time_m};
    return res;
}

// Default models of controller: (no controller)
float* controller(Satellite sat)
{
    float* res = {0.0, 0.0, 0.0};
    return res;
}

// Default models of environment: (no disturbance)
float* disturbance(Satellite sat)
{
    float* res = {0.0, 0.0, 0.0};
    return res;
}

// Default models of estimator: (returns qBO obtained by integrator)
quat estimator(Satellite sat)
{
    return (sat.getQ_BO());
}
