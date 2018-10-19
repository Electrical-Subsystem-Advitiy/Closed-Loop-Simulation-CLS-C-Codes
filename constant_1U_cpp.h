#include<iostream>
#include<math.h>
#include<time.h>
#include "qnv_cpp.h"
#include "matrix.h"

using namespace std;

double pi = 3.1415926535897932384626433;

// Earth and environment
float w_earth = 7.2921150*pow(10,-5); // rotation velocity of the earth (rad per second)
float G = 6.67408*pow(10,-11);        // universal gravitational constant, SI
double M_earth = 5.972*pow(10,24);     // mass of earth, kg
float R_earth = 6371.0*pow(10,3);     // radius of earth, m
float altitude = 700.0*pow(10,3);       // (in m) assuming height of satellite 700 km
float r = R_earth + altitude;         // Distance of satellite from center of earth m

float v_w_IO_o[3] = {0.0, sqrt(G*M_earth/(pow(r, 3))), 0.0}; // angular velocity of inertial frame wrt orbit frame in orbit frame

float AU = 149597870700.0;            // Distance between sun and earth in meters
float R_sun = 6957.0*pow(10, 5);      // Radius of the Sun in meters

/*
// date format yyyy,mm,dd (TLE taken from n2yo.com on 3rd April)
LINE1 = ('1 41783U 16059A   18093.17383152  .00000069  00000-0  22905-4 0  9992') #Insert TLE Here
LINE2 = ('2 41783  98.1258 155.9141 0032873 333.2318  26.7186 14.62910114 80995')
Incl = LINE2[8:16]
Inclination = float("".join(map(str, Incl)))

TPer = LINE2[52:63];
TiPer = float("".join(map(str, TPer)));
time_t TimePeriod = 86400/TiPer;
*/

time_t EPOCH = 1522759819;    // date of launch t=0 (2018, 4, 3, 12, 50, 19)
time_t EQUINOX = 1521551100;	// day of equinox (2018, 3, 20, 13, 5, 00)
float STEPRUT = 1.002738; // sidereal time = steprut * universal time

// Moment of inertia matrix in kg m^2 for 1U satellite (assumed to be uniform with small off-diagonal) (wrt center of mass)
float MASS_SAT = 0.79211825320; // in kg
float Lx = 0.1;                 // in meters
float Ixx = 0.00152529;
float Iyy = 0.00145111;
float Izz = 0.001476;
float Ixy = 0.00000437;
float Iyz = -0.00000408;
float Ixz = 0.00000118;

float matrix[3][3] = {{Ixx, Ixy, Ixz}, {Ixy, Iyy, Iyz}, {Ixz, Iyz, Izz}};
matrix3 m_INERTIA = matrix3(matrix);        // actual inertia
// m_INERTIA.mat = 0.001*{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};  // identity inertia

matrix3 m_INERTIA_inv = invert_mat(m_INERTIA);	// inverse of inertia matrix
/*
J=np.linalg.eig(m_INERTIA);
Jmin = min(J[0][0],J[0][1],J[0][2]);
*/

// Side panel areas
float v_Ax[3] = {0.01, 0.0, 0.0};       // area vector perpendicular to x-axis in m^2
float v_Ay[3] = {0.0, 0.01, 0.0};       // area vector perpendicular to y-axis in m^2
float v_Az[3] = {0.0, 0.0, 0.01};       // area vector perpendicular to z-axis in m^2

// Initial conditions
float dummy[4] = {1.0,0.0,0.0,0.0};
quat v_q0_BO = quat(dummy); // unit quaternion initial condition

float MODEL_STEP = 0.1;
float CONTROL_STEP = 2.0;   // control cycle time period in second
float h = 0.001;            // step size of integration in seconds

float INDUCTANCE = 68.0*pow(10, -3);        // Inductance of torquer in Henry
float RESISTANCE = 107.0;                   // Resistance of torquer in Ohm
float PWM_AMPLITUDE = 3.3;                  // PWM amplitude in volt
float PWM_FREQUENCY = 1000.0;               // frequency of PWM signal
int No_Turns = 450;                         // No. of turns of torquer
float v_A_Torquer[3] = {0.0049,0.0049,0.0049}; // area vector of torquers in m^2

// Disturbance model constants
float SOLAR_PRESSURE = 4.56*pow(10, -6);    // in N/m^2
float REFLECTIVITY = 0.2;
float r_COG_2_COM_b[3] = {-0.69105608*pow(10, -3), -0.69173140*pow(10, -3), -2.37203930*pow(10, -3)};
float AERO_DRAG = 2.2;
double RHO = 0.218*pow(10, -12);

/*
k_detumbling = 4*pi*(1+sin(radians(Inclination-11)))*Jmin/TimePeriod;    // gain constant in B_dot controller (from book by F. Landis Markley)
*/
