#include<iostream>

using namespace std;

class matrix3
{
    public: float mat[3][3];
            matrix3()
            {
                for(int i = 0; i<3; i++)
                    for(int j = 0; j<3; j++)
                        mat[i][j] = 0;
            }
            matrix3(float mat1[3][3])
            {
                for(int i = 0; i<3; i++)
                    for(int j = 0; j<3; j++)
                        mat[i][j] = mat1[i][j];
            }
            matrix3(const matrix3 &matrix)
            {
                for(int i = 0; i<3; i++)
                    for(int j = 0; j<3; j++)
                        mat[i][j] = matrix.mat[i][j];
            }
            matrix3 operator+(matrix3 &a)
            {
                matrix3 temp;
                for(int i = 0; i<3; i++)
                    for(int j = 0; j<3; j++)
                        temp.mat[i][j] = mat[i][j] + a.mat[i][j];
                return temp;
            }
            matrix3 operator-(matrix3 &a)
            {
                matrix3 temp;
                for(int i = 0; i<3; i++)
                    for(int j = 0; j<3; j++)
                        temp.mat[i][j] = mat[i][j] - a.mat[i][j];
                return temp;
            }
};

matrix3 invert_mat(matrix3 a)
{
    matrix3 b;
    float det = 0;
    for(int i = 0; i<3; i++)
        det = det + (a.mat[0][i]*(a.mat[1][(i+1)%3]*a.mat[2][(i+2)%3] - a.mat[1][(i+2)%3]*a.mat[2][(i+1)%3]));
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            b.mat[j][i] = ((a.mat[(i+1)%3][(j+1)%3]*a.mat[(i+2)%3][(j+2)%3])-(a.mat[(i+1)%3][(j+2)%3]*a.mat[(i+2)%3][(j+1)%3])) / det;
    return b;
}
