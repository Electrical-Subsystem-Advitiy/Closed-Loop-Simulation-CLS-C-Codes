#include<iostream>
#include<math.h>
#include "frames_cpp.h"

using namespace std;

class Satellite
{
    float v_state[7];
    float time;
    float v_pos_i[3], v_vel_i[3];
    quat v_q_BI, v_q_BO;
    float v_w_BO_b[3], v_w_BI_b[3], v_dist_b[3], v_solar_dist_b[3], v_aero_dist_b[3], v_gg_dist_b[3], v_control_b[3], v_sun_i[3], v_sun_o[3], v_mag_o[3], v_mag_i[3], v_sun_b_m[3], mag_b_m_c[3], mag_b_m_p[3];
    quat quatEstimate;
    float v_w_BO_b_m[3];
    int light;
    float v_req_Magmoment_b[3], v_app_torque_b[3];
    float gpsData[7], J2Data[7];
    float GyroVarBias[3];

    public: Satellite(float v_state0[7], float time0)
            {
                setTime(time0);
                setState(v_state0);
            }
            void setState(float v_state1[7])                // set state
            {
                for(int i = 0; i<7; i++)
                    v_state[i] = v_state1[i];
            }
            float* getState()                               // return state
            {
                return v_state;
            }
            void setPos(float v_pos[3])                     // set position in eci (earth centered inertial frame)
            {
                for(int i = 0; i<3; i++)
                    v_pos_i[i] = v_pos[i];
            }
            float* getPos()                                 // return position in eci
            {
                return v_pos_i;
            }
            void setVel(float v_vel[3])                     // set velocity in eci
            {
                for(int i = 0; i<3; i++)
                    v_vel_i[i] = v_vel[i];
            }
            float* getVel()                                 // return velocity in eci
            {
                return v_vel_i;
            }
            void setQ_B0(quat v_w)                          // set error quaternion
            {
                for(int i = 0; i<4; i++)
                    v_state[i] = v_w.arr[i];
            }
            quat getQ_B0()                                  // return error quaternion
            {
                float dummy[4];
                for(int i = 0; i<4; i++)
                    dummy[i] = v_state[i];
                quat a = quat(dummy);
                return a;
            }
            quat getQ_BI()                                  // get exact quaternion from inertial frame to body frame
            {
                float dummy[4];
                for(int i = 0; i<4; i++)
                    dummy[i] = v_state[i];
                quat v_q_B = quat(dummy);
                quat v_q_BI = qBO2qBI(v_q_B, v_pos_i, v_vel_i);
                return v_q_BI;
            }

            void setW_B0_b(float v_w[3])                      // set angular velocity of Body frame wrt orbit frame in body frame
            {
                for(int i = 4; i<7; i++)
                    v_state[i] = v_w[i-4];
            }
            float* getW_B0_b()                              // return angular velocity of Body frame wrt orbit frame in body frame
            {
                static float a[3];
                for(int i = 4; i<7; i++)
                    a[i-4] = v_state[i];
                return a;
            }
            float* getW_BI_b()                              // return exact angular velocity of body frame with respect to inertial frame expressed in body frame
            {
                float v_w_BO_b[3];
                for(int i = 4; i<7; i++)
                    v_w_BO_b[i-4] = v_state[i];
                quat v_q_BO;
                for(int i = 0; i<4; i++)
                    v_q_BO.arr[i] = v_state[i];
                static float v_w_BI_b[3];
                for(int i = 0; i<3; i++)
                    v_w_BI_b[i] = (wBOb2wBIb(v_w_BO_b,v_q_BO,v_w_IO_o))[i];
                return v_w_BI_b;
            }
            void setTime(float y)                           // set time
            {
                time = y;
            }
            float getTime()                                 // return time
            {
                return time;
            }
            void setDisturbance_b(float v_torque_dist_b[3]) // set disturbance in body (about center of mass)
            {
                for(int i = 0; i<3; i++)
                    v_dist_b[i] = v_torque_dist_b[i];
            }
            float* getDisturbance_b()                       // return disturbance in body
            {
                return v_dist_b;
            }
            void setsolarDisturbance_b(float v_solar_torque_dist_b[3])
            {
                for(int i = 0; i<3; i++)
                    v_solar_dist_b[i] = v_solar_torque_dist_b[i];
            }
            float* getsolarDisturbance_b()
            {
                return v_solar_dist_b;
            }
            void setaeroDisturbance_b(float v_aero_torque_dist_b[3])
            {
                for(int i = 0; i<3; i++)
                    v_aero_dist_b[i] = v_aero_torque_dist_b[i];
            }
            float* getaeroDisturbance_b()
            {
                return v_aero_dist_b;
            }
            void setggDisturbance_b(float v_gg_torque_dist_b[3])
            {
                for(int i = 0; i<3; i++)
                    v_gg_dist_b[i] = v_gg_torque_dist_b[i];
            }
            float* getggDisturbance_b()
            {
                return v_gg_dist_b;
            }
            void setControl_b(float v_control[3])           // set control torque in body
            {
                for(int i = 0; i<3; i++)
                    v_control_b[i] = v_control[i];
            }
            float* getControl_b()                           // return control torque in body
            {
                return v_control_b;
            }
            void setSun_i(float v_sv_i[3])                  // set sun vector in eci
            {
                for(int i = 0; i<3; i++)
                    v_sun_i[i] = v_sv_i[i];
            }
            float* getSun_i()                               // return sun vector in eci
            {
                return v_sun_i;
            }
            float* getSun_o()                               // return sun vector in orbit
            {
                v_sun_o = ecif2orbit(v_pos_i, v_vel_i, v_sun_i);
                return v_sun_o;
            }
            void setMag_i(float v_mg_i[3])                  // set mag in eci
            {
                for(int i = 0; i<3; i++)
                    v_mag_i[i] = v_mg_i[i];
            }
            float* getMag_i()                               // return mag in eci
            {
                return v_mag_i;
            }
            float* getMag_o()                               // return mag in orbit
            {
                v_mag_o = ecif2orbit(v_pos_i, v_vel_i, v_mag_i);
                return v_mag_o;
            }
            void setSun_b_m(float v_sv_b_m[3])              // set sunsensor measurement in body
            {
                for(int i = 0; i<3; i++)
                    v_sun_b_m[i] = v_sv_b_m[i];
            }
            float* getSun_b_m()                             // return sunsensor measurement in body
            {
                return v_sun_b_m;
            }
            void setMag_b_m_c(float v_mag_b_m[3])           // set current mag measurement in body
            {
                for(int i = 0; i<3; i++)
                    mag_b_m_c[i] = v_mag_b_m[i];
            }
            float* getMag_b_m_c()                           // return mag measurement in body
            {
                return mag_b_m_c;
            }
            void setMag_b_m_p(float v_mag_b_m[3])           // set previous mag measurement in body
            {
                for(int i = 0; i<3; i++)
                    mag_b_m_p[i] = v_mag_b_m[i];
            }
            float* getMag_b_m_p()                           // return mag measurement in body
            {
                return mag_b_m_p;
            }
            void setQUEST(quat v_q_BO_m)                    // set quest quaternion
            {
                for(int i = 0; i<4; i++)
                    quatEstimate.arr[i] = v_q_BO_m.arr[i];
            }
            quat getQUEST()                                 // return quest quaternion
            {
                return quatEstimate;
            }
            void setOmega_m(float omega_m[3])
            {
                for(int i = 0; i<3; i++)
                    v_w_BO_b_m[i] = omega_m[i];
            }
            float* getOmega_m()
            {
                return v_w_BO_b_m;
            }
            void setLight(int flag)
            {
                light = flag;
            }
            int getLight()
            {
                return light;
            }
            void setMagmomentRequired_b(float v_rq_Magmoment_b[3])      // set applied torque
            {
                for(int i = 0; i<3; i++)
                    v_req_Magmoment_b[i] = v_rq_Magmoment_b[i];
            }
            float* getMagmomentRequired_b()                         // get applied torque
            {
                return v_req_Magmoment_b;
            }
            void setAppTorque_b(float v_app_torque[3])      // set applied torque
            {
                for(int i = 0; i<3; i++)
                    v_app_torque_b[i] = v_app_torque[i];
            }
            float* getAppTorque_b()                         // get applied torque
            {
                return v_app_torque_b;
            }
            void setgpsData(float gpsdata[7])               // gpsdata is an array of 7 elements containing v_pos_m,v_vel_m,time_m in sequence
            {
                for(int i = 0; i<7; i++)
                    gpsData[i] = gpsdata[i];
            }
            float* getgpsData()                             // get gpsData
            {
                return gpsData;
            }
            void setJ2Data(float J2data[7])                 // J2data is an array of 7 elements containing v_pos_m,v_vel_m,time_m in sequence
            {
                for(int i = 0; i<7; i++)
                    J2Data[i] = J2data[i];
            }
            float* getJ2Data()                              // get J2Data
            {
                return J2Data;
            }
            void setGyroVarBias(float v_gyro_bias[3])       // set bias of gyroscope
            {
                for(int i = 0; i<3; i++)
                    GyroVarBias[i] = v_gyro_bias[i];
            }
            float* getGyroVarBias()                         // get bias of gyroscope
            {
                return GyroVarBias;
            }
};
