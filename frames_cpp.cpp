#include<iostream>
#include<math.h>

using namespace std;

float w_earth = 7.2921159*pow(10,-5);

float dot(float a[3], float b[3])
{
    float res = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    return res;
}

float* cross(float a[3], float b[3])
{
    static float res[3];
    res[0] = a[1]*b[2] - b[1]*a[2];
    res[1] = a[2]*b[0] - b[2]*a[0];
    res[2] = a[0]*b[1] - b[0]*a[1];
    return res;
}

float radians(float deg)
{
    float rad = deg*0.0174532925;
    return rad;
}

float* ecef2ecif(float* x, float t)
{
    float theta = w_earth*t;
	float DCM[3][3] = {{cos(theta), -1*sin(theta), 0}, {sin(theta), cos(theta),0}, {0,0,1}};
	static float y[3];
	for(int i = 0; i<3; i++)
	    y[i] = dot(DCM[i],x);
	return y;
}

float* ned2ecef(float* x, float lat, float lon)
{
    float v[3] = {-x[2], -x[0], x[1]};   // convert to spherical polar r theta phi
	float theta = -lat + 90;    // in degree, polar angle
	float phi = lon;          // in degree, azimuthal angle
	theta = radians(theta);
	phi = radians(phi);
	float DCM[3][3] = {{sin(theta)*cos(phi), cos(theta)*cos(phi), -sin(phi)}, {sin(theta)*sin(phi), cos(theta)*sin(phi), cos(phi)}, {cos(theta), -sin(theta), 0}}; //spherical to cartesian
	static float y[3];
    for(int i = 0; i<3; i++)
	    y[i] = dot(DCM[i],v);
	return y;
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
    cout<<"Hello world!"<<endl;
    return 0;
}
