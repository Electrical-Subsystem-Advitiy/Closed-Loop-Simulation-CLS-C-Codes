/*
	This code generates csv file for latitude, longitude, altitude (LLA) corresponding to time data.
	ECI frame position is provided from sgp_output.csv
		Time: (since epoch) in seconds
		latitude: -90 to 90 degrees
		longitude: -180 to 180 degrees (-180 excluded)
		altitude: in meters
*/

#include<iostream>
#include<math.h>
#include<time.h>
#include "frames_cpp.h"

float* getLLA(float m_sgp_output_i[4])
{

    float m_sgp_ecef[4];
    static float m_LLA[4];
    for(int i = 0; i<4; i++)
    {
        m_sgp_ecef[i] = 0;
        m_LLA[i] = 0;
    }

    float v_i[3];		                    // inertial frame position
    for(int i = 0; i<3; i++)
        v_i[i] = m_sgp_output_i[i+1];
    float time = m_sgp_output_i[0];     	// time in sec

    // get position in ecef
    float v_ecef[3] = {0.0, 0.0, 0.0};
    for(int i = 0; i<3; i++)
        v_ecef[i] = (ecif2ecef(v_i,time))[i];

    // get latitude and longitude and altitude
    float latt = (latlon(v_ecef))[0], lonn = (latlon(v_ecef))[1];
    float alt = norm(v_i) - R_earth;    // in meters

    m_sgp_ecef[0] = time;
    for(int i = 0; i<3; i++)
        m_sgp_ecef[i+1] = v_ecef[i];

    m_LLA[0] = time;
    m_LLA[1] = latt;
    m_LLA[2] = lonn;
    m_LLA[3] = alt;

    return m_LLA;
}
