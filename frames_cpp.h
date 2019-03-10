#include<iostream>
#include<math.h>
#include<time.h>
#include<stdexcept>
#include "constants_1U_cpp.h"

using namespace std;

float* latlon(float v_x[3])
{
	// get the latitude and longitude in degrees given position in ECEF
	// Input: x is position in ecef frame, latitude and longitude variables in degrees (tuple)
	// latitude ranges [0,90] in north hemisphere and [0,-90] in south hemisphere
    float lat = sgn(v_x[2])*acos(pow((pow(v_x[0], 2) + pow(v_x[1], 2)), 0.5)/(pow(pow(v_x[0], 2) + pow(v_x[1], 2 + pow(v_x[2], 2)), 0.5)))*90.0/(pi/2.0);

    // longitude calculation given position, lon is longitude
	// ranges from (-pi,pi]
    float lon;
    if(v_x[1] == 0)
    {
        if(v_x[0] >= 0)
            lon = 0.0;
        else
            lon = 180.0;
    }
    else
        lon = sgn(v_x[1])*acos(v_x[0]/(pow((pow(v_x[0], 2) + pow(v_x[1], 2)), 0.5)))*90.0/(pi/2);
    // x axis is intersection of 0 longitude and 0 latitude
    static float lattlonn[2] = {lat, lon};
    return lattlonn;
}

float* ecif2ecef(float* v_x_i, time_t t)
{
    // Input ecif vector and time since epoch in seconds
	// Output ecef vector
    time_t ut_sec = (EPOCH - EQUINOX) + t; // universal time vector in sec
	float theta = w_earth*ut_sec;
	float m_DCM[3][3] = {{cos(theta), sin(theta), 0.0}, {-1*sin(theta), cos(theta),0.0}, {0.0,0.0,1.0}};
	static float v_x_e[3];
	for(int i = 0; i<3; i++)
	    v_x_e[i] = dot(m_DCM[i],v_x_i);
	return v_x_e;
}

/*
float** ecif2ecefR(float t):
{
    float ut_sec = (EPOCH - EQUINOX) + t;                               // universal time vector in sec
	float st_sec = STEPRUT*ut_sec;                                      // sidereal time vector in sec
	float phi = st_sec*W_EARTH;                                         // sidereal time vector in rad
    float** TEI = {{cos(phi), sin(phi), 0.0}, {-sin(phi), cos(phi), 0.0}, {0.0, 0.0, 1.0}};

	return TEI
}
*/

float* ecef2ecif(float v_x_e[3], time_t t)
{
    // Input ecef vector and time since epoch in seconds
	// Output ecif vector
    time_t ut_sec = (EPOCH - EQUINOX) + t;
    float theta = w_earth*ut_sec;
	float DCM[3][3] = {{cos(theta), -1*sin(theta), 0.0}, {sin(theta), cos(theta),0.0}, {0.0,0.0,1.0}};
	static float v_x_i[3];
	for(int i = 0; i<3; i++)
	    v_x_i[i] = dot(DCM[i], v_x_e);
	return v_x_i;
}

float* ecif2orbit(float v_pos_i[3], float v_vel_i[3], float v_x_i[3])
{
    // Input: v_pos_i is position in eci frame , v_vel_i is velocity in eci frame, v_x_i is vector to be transformed
	// Output: vector components in orbit frame
	float z[3], y[3], x[3];
	for(int i = 0; i<3; i++)
    {
        z[i] = - (v_pos_i[i])/v_pos_i.norm();
        y[i] = (cross(v_vel_i, v_pos_i))[i];
    }
    y = y/y.norm;
	for(int i = 0; i<3; i++)
        x[i] = (cross(y, z))[i];
	x = x/x.norm;

###	float m_DCM_OI[3][3] = {x, y, z};
	float v_x_o[3];
	for(int i = 0; i<3; i++)
        v_x_o[i] = dot(m_DCM_OI[i], v_x_i);

	return v_x_o;
}

quat qBI2qBO(quat v_q_BI, float v_pos_i[3], float v_vel_i[3])
{
    // input: unit quaternion which rotates ecif vector to body frame, position and velocity in ecif
	// output: unit quaternion which rotates orbit frame vector to body frame
	// Orbit frame def:	z_o = nadir(opposite to satellite position vector) y_o: cross(v,r) x_o: cross(y,z)

	float z[3], y[3], x[3];
	for(int i = 0; i<3; i++)
    {
        z[i] = - (v_pos_i[i])/v_pos_i.norm();
        y[i] = (cross(v_vel_i, v_pos_i))[i];
    }
    y = y/y.norm;
	for(int i = 0; i<3; i++)
        x[i] = (cross(y, z))[i];
	x = x/x.norm;

	float temp[3][3] = {x, y, z};
    matrix3 m_DCM_OI = matrix3(temp);

	quat v_q_IO = rotm2quat(transpose(m_DCM_OI));
	quat v_q_BO = quatMultiplyNorm(v_q_BI, v_q_IO);
	if (v_q_BO[3] < 0.0)
		v_q_BO = v_q_BO * (-1);
	v_q_BO = v_q_BO/v_q_BO.norm();

	return v_q_BO
}

quat qBO2qBI(quat v_q_BO, float v_pos_i[3], float v_vel_i[3]):
{
    // input: Unit quaternion which rotates orbit frame vector to body frame, position and velocity in ecif
	// output: Unit quaternion which rotates ecif vector to body frame
	// Orbit frame def:	z_o = nadir(opposite to satellite position vector) y_o: cross(v,r) x_o: cross(y,z)

	float z[3], y[3], x[3];
	for(int i = 0; i<3; i++)
    {
        z[i] = - (v_pos_i[i])/v_pos_i.norm();
        y[i] = (cross(v_vel_i, v_pos_i))[i];
    }
    y = y/y.norm;
	for(int i = 0; i<3; i++)
        x[i] = (cross(y, z))[i];
	x = x/x.norm;

	float temp[3][3] = {x, y, z};
    matrix3 m_DCM_OI = matrix3(temp);

	v_q_OI = rotm2quat(m_DCM_OI);
	v_q_BI = quatMultiplyNorm(v_q_BO, v_q_OI);
	if (v_q_BI[3] < 0.0)
		v_q_BI = v_q_BI * (-1);
	v_qBI = v_q_BI/v_q_BI.norm();

	return v_q_BI
}

float* wBIb2wBOb(float v_w_BI_b[3], quat v_q_BO, float v_w_IO_o[3])
{
// input: angular velocity of body wrt ecif in body frame, unit quaternion which rotates orbit vector to body frame and angular velocity of ecif wrt orbit frame in orbit frame
// output: angular velocity of body frame wrt orbit frame in body frame
    static float v[3];
    for(int i = 0; i<3; i++)
        v[i] = v_w_BI_b[i] - (quatRotate(v_q_BO, v_w_IO_o))[i];
    return v;
}

float* ned2ecef(float* v, float lat, float lon)
{
	//  rotate vector from North-East-Down frame to ecef
	//  Input:
	//	lat: latittude in degrees ranges from -90 to 90
	//	lon: longitude in degrees ranges from (-180,180]
    if(lat==90||lat==-90)
        throw invalid_argument("Latittude value +/-90 occured. NED frame is not defined at north and south pole !!");
    float theta = -lat + 90.0;          // in degree, polar angle (co-latitude)
    float phi;
    if(lon<0)
        phi = 360.0 - lon*sgn(lon);
    else
        phi = lon;                      // in degree, azimuthal angle
    theta = radians(theta);
    phi = radians(phi);

	float m_DCM_n2e[3][3] = {{-1*cos(theta)*cos(phi), -1*sin(phi), -sin(theta)*cos(phi)}, {-1*cos(theta)*sin(phi), cos(phi), -1*sin(theta)*sin(phi)}, {sin(theta), 0.0, -cos(theta)}}; //spherical to cartesian
	static float y[3];
    for(int i = 0; i<3; i++)
	    y[i] = dot(m_DCM_n2e[i], v);
	return y;
}

float* wBOb2wBIb(float v_w_BO_b[3], quat v_q_BO, float v_w_IO_o[3])
{
// input: angular velocity of body wrt orbit in body frame, unit quaternion which rotates orbit vector to body frame and angular velocity of ecif wrt orbit frame in orbit frame
// output: angular velocity of body frame wrt eci frame in body frame
    static float v[3];
    for(int i = 0; i<3; i++)
        v[i] = v_w_BO_b[i] - (quatRotate(v_q_BO, v_w_IO_o))[i];
    return v;
}
