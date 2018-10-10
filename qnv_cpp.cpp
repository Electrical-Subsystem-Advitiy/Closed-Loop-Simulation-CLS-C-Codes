#include<iostream>
#include<math.h>

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
            friend quat quatMultiplyUnnorm(quat v_q1, quat v_q2);
            friend float* quatRotate (quat a, quat b);
            friend quat dot (quat a, quat b);
            friend ostream & operator << (ostream &out, const quat &q);
            friend istream & operator >> (istream &in,  quat &q);
};

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
    static float v_q2[3];
    for(int i = 0; i<3; i++)
        v_q2[i] = v_y.arr[i];
    return v_q2;
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

int main()
{
    quat a;
    float b[3] = {0, 0, 0};
    float c[3] = {1.0, 0.0, 0.0};
    float d[3];
    float e[3];
    cin>>a;
    for(int i = 0; i<3; i++)
    {
        d[i] = (quatRotate(a, b))[i];
        e[i] = (quatRotate(a, c))[i];
    }
    cout<<a.norm()<<endl<<a.quatInv()<<endl<<dot(c, c)<<endl;
    for(int i = 0; i<3; i++)
        cout<<d[i]<<' ';
    cout<<endl;
    for(int i = 0; i<3; i++)
        cout<<e[i]<<' ';
}
