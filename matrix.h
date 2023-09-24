#include <vector>
#include <ctime>
#include <math.h>
#include <iostream>
using namespace std;
class matrix
{
public:
	double** a;
	int m, n;
	matrix() {};
	matrix(int m, int n)
	{
		this->m = m; 
		this->n = n;
		a = new double* [m]; 
		for (int i = 0; i < m; i++)
		{
			a[i] = new double[n]; 
			for (int j = 0; j < n; j++)
			{
				if (i == j)
				{
					a[i][j] = 1;
				}
				else
				a[i][j] = 0;
			}
		}
	}
	matrix(int m, int n, double RandNum)
	{
		srand(time(NULL));
		this->m = m; 
		this->n = n;
		a = new double* [m]; 
		for (int i = 0; i < m; i++)
		{
			a[i] = new double[n]; 
			for (int j = 0; j < n; j++)
			{
				RandNum =rand() %10;
				RandNum = RandNum / 10 + 1;
				a[i][j] = RandNum;
			}
		}
	}
public:
	double GetMatrix(int i, int j)
	{
		return a[i][j];
	}
public:

	double SetMatrix(int i, int j, double value)
	{
		return a[i][j] = value;
	}
	void ShowMatrix()
	{
		for (int i = 0; i < m; i++)
		{
			for (int j = 0; j < n; j++)
			{
				cout << a[i][j] << " ";
			}
			cout <<  endl;
		}
	}
	matrix transpond(matrix a)
	{
		int atn, atm;
		atn = a.m;
		atm = a.n;
		matrix at(atm, atn);
		for (int i = 0; i < atm; i++)
		{
			for (int j = 0; j < atn; j++)
			{
				at.SetMatrix(i, j, a.GetMatrix(j, i));
			}
		}
		return at;
	}
	matrix multiply(matrix first, matrix second){
		matrix result(first.m, second.n);
        for (int i = 0; i < first.m; i++) {
            for (int j = 0; j < second.n; j++) {
                float sum = 0.0;
                for (int k = 0; k < first.n; k++) {
                    sum += first.GetMatrix(i, k) * second.GetMatrix(k,j);
                }
                result.SetMatrix(i, j, sum);
            }
	}
	return result;
	}
	~matrix() {};
};
class DHMatrix : public matrix {
	DHMatrix() : matrix() {};
public :
	DHMatrix(int m, int n) : matrix(m, n) {};
	void setTheta(vector<float> Theta){
    	for (int j = 0; j < m; j++)
    	{
        SetMatrix(j,0, Theta[j]);
    	}
	}
	double getTheta(int i){
		return GetMatrix(i, 0);
	}
	void set_a(vector<float> a){
    	for (int j = 0; j < m; j++)
    	{
        SetMatrix(j,1, a[j]);
    	}
	}
	double get_a(int i){
		return GetMatrix(i,1);
	}
	void set_d(vector<float> d){
    	for (int j = 0; j <  m; j++)
    	{
        SetMatrix(j,2, d[j]);
    	}
	}
	double get_d(int i){
		return GetMatrix(i,2);
	}
	void set_alpha(vector<float> alpha){
    	for (int j = 0; j < m; j++)
    	{
        SetMatrix(j,3, alpha[j]);
    	}
	}
	double get_alpha(int i){
		return GetMatrix(i,3);
	}
	DHMatrix(int DOF,vector<float>Theta, vector<float>a, vector<float>d, vector<float>alpha) : matrix(DOF, 4)
	{
		setTheta(Theta);
    	set_a(a);
    	set_d(d);
    	set_alpha(alpha);
	}
	matrix createTmatrix(int i)
	{
		matrix T(4,4);
		T.SetMatrix(0,0, cos(getTheta(i)));
		T.SetMatrix(0,1, -(cos(get_alpha(i))*sin(getTheta(i))));
		T.SetMatrix(0,2, sin(get_alpha(i))*sin(getTheta(i)));
		T.SetMatrix(0,3, get_a(i)*cos(getTheta(i)));
		T.SetMatrix(1,0, sin(getTheta(i)));
		T.SetMatrix(1,1, cos(get_alpha(i))*cos(getTheta(i)));
		T.SetMatrix(1,2, -(sin(get_alpha(i))*cos(getTheta(i))));
		T.SetMatrix(1,3, get_a(i)*sin(getTheta(i)));
		T.SetMatrix(2,0, 0);
		T.SetMatrix(2,1, sin(get_alpha(i)));
		T.SetMatrix(2,2, cos(get_alpha(i)));
		T.SetMatrix(2,3, get_d(i));
		T.SetMatrix(3,0, 0);
		T.SetMatrix(3,1, 0);
		T.SetMatrix(3,2, 0);
		T.SetMatrix(3,3, 1);
		
		return T;
	}
	matrix forward_kinematics()
	{
		matrix A(4,4);
		for (int i = 0; i < m; i++)
		{
			A = A.multiply(A,createTmatrix(i));
		}
		return A;
	}
	~DHMatrix(){};
};