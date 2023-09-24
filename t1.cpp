#pragma once
#include "matrix.h"
using namespace std;

int main(){
    vector<float> Theta;
    //vector<float> Theta = {10, -50, -60, 90, 50, 0};
    vector<float> a = {0, -0.8, -0.598, 0, 0, 0};
    vector<float> d = {0.21, 0.193, -0.16, 0.25, 0.25, 0.25};
    vector<float> alpha = {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0};
    // при вводе не с клавиатуры, следует удалить цикл for и раскомментировать строчку выше
    float tmp;
    for (int i = 0; i < 6; i++)
    {
        cout << "введите " << i+1 << "-й элемент вектора обобщённых координат" << endl;
        cin >>  tmp;
        Theta.push_back(tmp);
    }
    
    DHMatrix m1(6,Theta,a,d,alpha);
    //m1.ShowMatrix();
    matrix Tm1 = m1.forward_kinematics();
    cout << "|||||" <<endl;
    double* pos = new double[3]; 
    pos[0] = Tm1.GetMatrix(0,3);
    pos[1] = Tm1.GetMatrix(1,3);
    pos[2] = Tm1.GetMatrix(2,3);
    cout<< "X = " << pos[0] << ";" << endl;
    cout<< "Y = " << pos[1] << ";"<< endl;
    cout<< "Z = " << pos[2] << ";"<< endl;     
}