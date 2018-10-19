#include<iostream>
#include<math.h>
#include<time.h>
#include "constants_1U_cpp.h"

using namespace std;

float radians(float deg)
{
    float rad = deg*0.0174532925;
    return rad;
}

int sgn(float num)
{
    if(num < 0)
        return -1;
    else if(num == 0)
        return 0;
    else
        return 1;
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

void latlon(float* v_x, float &lat, float &lon)
{
	// get the latitude and longitude in degrees given position in ECEF
	// Input: x is position in ecef frame, latitude and longitude variables in degrees (tuple)
	// latitude ranges [0,90] in north hemisphere and [0,-90] in south hemisphere
    lat = sgn(v_x[2])*acos(pow((pow(v_x[0], 2) + pow(v_x[1], 2)), 0.5)/(pow(pow(v_x[0], 2) + pow(v_x[1], 2 + pow(v_x[2], 2)), 0.5)))*90.0/(pi/2.0);

    // longitude calculation given position, lon is longitude
	// ranges from (-pi,pi]
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
}

float* ecef2ecif(float* v_x_e, time_t t)
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

float* wBIb2wBOb(float v_w_BI_b[3], quat v_q_BO, float v_w_IO_o[3])
{
// input: angular velocity of body wrt ecif in body frame, unit quaternion which rotates orbit vector to body frame and angular velocity of ecif wrt orbit frame in orbit frame
// output: angular velocity of body frame wrt orbit frame in body frame
    static float v[3];
    for(int i = 0; i<3; i++)
        v[i] = v_w_BI_b[i] - (quatRotate(v_q_BO, v_w_IO_o))[i];
    return v;
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

int main()
{
    float x[3];
    float y[3];
    float lat, lon, t;
    cout<<"\n ecef2ecif Test";
    for(int i = 0; i<3; i++)
        cin>>x[i];
    cin>>t; cout<<endl;
    for(int i = 0; i<3; i++)
        y[i] = (ecef2ecif(x, t))[i];
    for(int i = 0; i<3; i++)
        cout<<y[i]<<' ';
    cout<<"\n ned2ecef Test";
    for(int i = 0; i<3; i++)
        cin>>x[i];
    cin>>lat>>lon; cout<<endl;
    for(int i = 0; i<3; i++)
        y[i] = (ned2ecef(x, lat, lon))[i];
    for(int i = 0; i<3; i++)
        cout<<y[i]<<' ';
    return 0;
}
