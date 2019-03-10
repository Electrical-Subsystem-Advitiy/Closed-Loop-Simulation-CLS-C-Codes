#include<iostream>
#include<math.h>
#include "matrix.h"

// A new quaternion class named quat has been defined.
// Completed functions: quatInv, quatMultiplyNorm, quatMultiplyUnnorm, quatRotate, quatDerBI, quatDerBO, rotm2quat, quat2rotm, quat2euler[, dot, cross, norm, sgn, radians]

using namespace std;

class quat
{
    public: float arr[4];
            quat()
            {
                for(int i = 0; i<4; i++)
                    arr[i] = 0;
            }
            quat(float arr1[4])
            {
                for(int i = 0; i<4; i++)
                    arr[i] = arr1[i];
            }
            quat(const quat &q)
            {
                for(int i = 0; i<4; i++)
                    arr[i] = q.arr[i];
            }
            quat operator+(quat &a)
            {
                quat temp;
                for(int i = 0; i<4; i++)
                    temp.arr[i] = arr[i] + a.arr[i];
                return temp;
            }
            quat operator-(quat &a)
            {
                quat temp;
                for(int i = 0; i<4; i++)
                    temp.arr[i] = arr[i] - a.arr[i];
                return temp;
            }
            quat operator*(double a)
            {
                quat temp;
                for(int i = 0; i<4; i++)
                    temp.arr[i] = arr[i] * a;
                return temp;
            }
            quat operator/(double a)
            {
                quat temp;
                for(int i = 0; i<4; i++)
                    temp.arr[i] = arr[i] / a;
                return temp;
            }
            quat quatInv()
            {
                quat v_qi;
                for(int i = 0; i<3; i++)
                    v_qi.arr[i] = - arr[i];
                v_qi.arr[3] = arr[3];
                return v_qi;
            }
            float norm()
            {
                float res = 0;
                for(int i = 0; i<4; i++)
                    res += arr[i]*arr[i];
                return res;
            }
            friend quat quatMultiplyNorm(quat v_q1, quat v_q2);
            friend quat quatMultiplyUnnorm(quat v_q1, quat v_q2);
            friend float* quatRotate (quat a, quat b);
            friend float dot (quat a, quat b);
            friend ostream & operator << (ostream &out, const quat &q);
            friend istream & operator >> (istream &in,  quat &q);
};

float dot(quat a, quat b)
{
    float res = 0;
    for(int i = 0; i<4; i++)
        res += a.arr[i]*b.arr[i];
    return res;
}

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

float norm(float* v_i)
{
    float val = 0;
    for(int i = 0; i<3; i++)
        val += v_i[i]*v_i[i];
    val = sqrt(val);
    return val;
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

quat quatMultiplyNorm(quat v_q1, quat v_q2)
{
    // returns quaternion product (product is a unit quaternion)

	float a1 = v_q1.arr[3];
	float a2 = v_q2.arr[3];
	float v_b1[3] = {v_q1.arr[0], v_q1.arr[1], v_q1.arr[2]}, v_b2[3] = {v_q2.arr[0], v_q2.arr[1], v_q2.arr[2]};

	float a = a1*a2 - dot(v_b1, v_b2);
	float v_b[3];
	for(int i = 0; i<3; i++)
        v_b[i] = a1*v_b2[i] + a2*v_b1[i] - (cross(v_b1, v_b2))[i];

	float v_q_float[4] = {a, v_b[0], v_b[1], v_b[2]};
	quat v_q = quat(v_q_float);
	v_q = v_q/v_q.norm();

	return v_q;
}

quat quatMultiplyUnnorm(quat v_q1, quat v_q2)
{
    float dot(float a[3], float b[3]);
    float* cross(float a[3], float b[3]);

    float a1 = v_q1.arr[3];
    float a2 = v_q2.arr[3];

    float v_b1[3], v_b2[3];
    for(int i = 0; i<3; i++)
    {
        v_b1[i] = v_q1.arr[i];
        v_b2[i] = v_q2.arr[i];
    }

    float a = a1*a2 - dot(v_b1, v_b2);
    float v_b[3];
    for(int i = 0; i<3; i++)
        v_b[i] = a1*v_b2[i] + a2*v_b1[i] - (cross(v_b1, v_b2))[i];
    float v_qa[4] = {v_b[0], v_b[1], v_b[2], a};
    quat v_q = quat(v_qa);
    return v_q;
}

float* quatRotate(quat v_q, float v_x[3])
{
    float dot(float a[3], float b[3]);
    if(dot(v_x, v_x) == 0)
        return v_x;
    quat v_qi = v_q.quatInv();
    float v_y1[4] = {v_x[0], v_x[1], v_x[2], 0.0};
    quat v_y = quat(v_y1);
    v_y = quatMultiplyUnnorm(v_q, v_y);
    v_y = quatMultiplyUnnorm(v_y, v_qi);
    static float v[3];
    for(int i = 0; i<3; i++)
        v[i] = v_y.arr[i];
    return v;
}

matrix3 quat2rotm(quat v_q)
{
    // given a quaternion it returns a rotation matrix
	float q1 = v_q.arr[0];
	float q2 = v_q.arr[1];
	float q3 = v_q.arr[2];
	float q4 = v_q.arr[3];

	float M1[3][3] = {{-q2*2 - q3*2, q1*q2, q1*q3}, {q1*q2, -q1*2 - q3*2, q2*q3}, {q1*q3, q2*q3, -q1*2 - q2*2}}, M2[3][3] = {{0,-q3,q2}, {q3,0,-q1}, {-q2,q1,0}}, M3[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
	matrix3 m_M1 = matrix3(M2); m_M1 = m_M1 * 2.0;
	matrix3 m_M2 = matrix3(M2); m_M2 = m_M2 * (-2.0 * q4);
	matrix3 m_M3 = matrix3(M3);                              // norm(v_q) = 1
	return (m_M1 + (m_M2 + m_M3));
}

float* quat2euler(quat v_q)
{
    // input quaternion
	// output Euler angles: roll, pitch, yaw in degrees
    float* m_M[3];
    for(int i = 0; i<3; i++)
        for(int j = 0; j<3; j++)
            m_M[i][j] = (quat2rotm(v_q)).mat[i][j];
	float yaw = (180/pi)*atan2(m_M[0][1], m_M[0][0]);
	float pitch = - (180/pi)*asin(m_M[0][2]);
	float roll = (180/pi)*atan2(m_M[1][2], m_M[2][2]);
	static float res[3] = {roll, pitch, yaw};
	return res;
}

quat quatDerBI(quat v_q, float v_w[3])
{
    // w is angular velocity of body wrt inertial frame in body frame.
    // q transforms inertial frame vector to body frame
    float m_W_float[4][4] = {{0.0, v_w[2], -v_w[1], v_w[0]}, {-v_w[2], 0.0, v_w[0], v_w[1]}, {v_w[1], -v_w[0], 0.0, v_w[2]}, {-v_w[0], -v_w[1], -v_w[2], 0.0}};
	quat m_W[4] = {quat(m_W_float[0]), quat(m_W_float[1]), quat(m_W_float[2]), quat(m_W_float[3])};
	quat v_q_dot;
	for(int i = 0; i<4; i++)
        v_q_dot.arr[i] = 0.5*dot(m_W[i],v_q);
	return v_q_dot;
}

quat quatDerBO(quat v_q, float v_w[3])          // q transforms orbit frame vector to body frame; w is angular velocity of body wrt orbit frame in body frame)
{
	float f1[4] = {0.0, v_w[2], -v_w[1], v_w[0]}, f2[4] = {-v_w[2], 0.0, v_w[0], v_w[1]}, f3[4] = {v_w[1], -v_w[0], 0.0, v_w[2]}, f4[4] = {-v_w[0], -v_w[1], -v_w[2], 0.0};
	quat m_W[4] = {quat(f1), quat(f2), quat(f3), quat(f4)};
	float a[4];
	for(int i = 0; i<4; i++)
        a[i] = 0.5*dot(m_W[i],v_q);
    quat v_q_dot = quat(a);
	return v_q_dot;
}

quat rotm2quat(matrix3 m_A)
{
    // returns a quaternion whose scalar part is positive to keep angle between -180 to +180 deg.
    // formula from pg 97 of textbook by Junkins
	float q4 = 1 + m_A.trace();
	float q1 = 1 + m_A.mat[0][0] - m_A.mat[1][1] - m_A.mat[2][2];
	float q2 = 1 - m_A.mat[0][0] + m_A.mat[1][1] - m_A.mat[2][2];
	float q3 = 1 - m_A.mat[0][0] - m_A.mat[1][1] + m_A.mat[2][2];
	float qm = max(max(max(q1, q2), q3), q4);
	if(qm == q4)
	{
	    q4 = sqrt(q4)/2.0;
		q1 = (m_A.mat[1][2] - m_A.mat[2][1])/(4.0*q4);
		q2 = (m_A.mat[2][0] - m_A.mat[0][2])/(4.0*q4);
		q3 = (m_A.mat[0][1] - m_A.mat[1][0])/(4.0*q4);
	}
	else if(qm == q1)
    {
    	q1 = sqrt(q1)/2.0;
		q4 = (m_A.mat[1][2] - m_A.mat[2][1])/(4.0*q1);
		q2 = (m_A.mat[0][1] + m_A.mat[1][0])/(4.0*q1);
		q3 = (m_A.mat[0][2] + m_A.mat[2][0])/(4.0*q1);
    }
	else if(qm == q2)
	{
	    q2 = sqrt(q2)/2.0;
		q4 = (m_A.mat[2][0] - m_A.mat[0][2])/(4.0*q2);
		q1 = (m_A.mat[0][1] + m_A.mat[1][0])/(4.0*q2);
		q3 = (m_A.mat[1][2] + m_A.mat[2][1])/(4.0*q2);
	}
	else
	{
	    q3 = sqrt(q3)/2.0;
		q4 = (m_A.mat[0][1] - m_A.mat[1][0])/(4.0*q3);
		q1 = (m_A.mat[0][2] + m_A.mat[2][0])/(4.0*q3);
		q2 = (m_A.mat[1][2] + m_A.mat[2][1])/(4.0*q3);
	}
	float temp[4] = {q1, q2, q3, q4};
	quat v_q = quat(temp);
	v_q = v_q/v_q.norm();
	if(sgn(v_q.arr[3]) == (-1))
		v_q = v_q * (double)(-1);
	return v_q;
}
ostream & operator << (ostream &out, const quat &q)
{
    out<<q.arr[0]<<' '<<q.arr[1]<<' '<<q.arr[2]<<' '<<q.arr[3]<<endl;
    return out;
}

istream & operator >> (istream &in,  quat &q)
{
    cout << "Enter the four values of the quaternion: ";
    in>>q.arr[0]>>q.arr[1]>>q.arr[2]>>q.arr[3];
    return in;
}

float radians(float deg)
{
    float rad = deg*0.0174532925;
    return rad;
}
