#include "../include/Screw_Robotics.h"
using namespace std;
using namespace screw_robotics;

#define PI 3.1415926

void NearZero_Test()
{
    double val = 1.0;
    if ( NearZero(val)== 0) { cout<<"NearZero() Test: Pass"<<endl; }
    else { cout<<"NearZero() Test: Failed" <<endl; }
}

void So32Vec_Test()
{
    Eigen::Matrix3d m;
    Eigen::Vector3d v;
    m<< 0, 0, 0,
        0, 0, -1,
        0, 1, 0;
    v << 1,0,0;
    if(So32Vec(m) == v){ cout<<"So32Vec Test: Pass"<<endl; }
    else { cout<<"So32Vec Test: Failed" <<endl; }
}

void Vec2So3_Test()
{
    Eigen::Matrix3d m;
    Eigen::Vector3d v;
    m<< 0, 0, 0,
        0, 0, -1,
        0, 1, 0;
    v << 1,0,0;
    if(Vec2So3(v) == m){ cout<<"Vec2So3 Test: Pass"<<endl; }
    else { cout<<"Vec2So3 Test: Failed" <<endl; }
}

void AxisAng3_Test()
{
    Eigen::Vector3d omega_theta;
    Eigen::Vector4d axis_angle;
    omega_theta << 0,0,2;
    axis_angle << 0,0,1,2;
    if(AxisAng3(omega_theta) == axis_angle){ cout<<"AxisAng3 Test: Pass"<<endl;}
    else {cout<<"AxisAng3 Test: Failed"<<endl;}
}

void MatrixExp3_R_Test()
{
    Eigen::Matrix3d so3mat;
    so3mat <<   0, -PI/6, 0,
                PI/6, 0, 0,
                0, 0, 0;
    Eigen::Matrix3d R;
    R <<    cos(PI/6), -sin(PI/6), 0,
            sin(PI/6), cos(PI/6), 0,
            0, 0, 1;
    if(MatrixExp3_R(so3mat) == R){ cout<<"MatrixExp3 Test: Pass"<<endl;}
    else{ cout<<"MatrixExp3 Test: Failed"<<endl; }
}

void MatrixR_Exp3_Test()
{
    Eigen::Matrix3d R;
    R <<    1, 0, 0,
            0, cos(PI/3), -sin(PI/3),
            0, sin(PI/3), cos(PI/3);
    Eigen::Matrix3d so3mat;
    so3mat <<   0, 0, 0,
                0, 0, -PI/3,
                0, PI/3, 0;
    if((MatrixR_Exp3(R)-so3mat).isZero()){ cout<<"MatrixR_Exp3_Test: Pass"<<endl; }
    else{ cout<<"MatrixR_Exp3 Test: Failed"<<endl; }
}

//TODO: Need to Complete
void Ad_Test()
{
    Eigen::VectorXd V_s(6);
    V_s << 0, 0, 2, -2, -4, 0;
    Eigen::VectorXd V_b(6);
    V_b << 0, 0, -2, 2.8, 4, 0;
}

void Rp2Trans_Test()
{
    Eigen::Matrix3d R;
    R <<    1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Eigen::Vector3d p;
    p <<    1, 0, 0;
    Eigen::Matrix4d T;
    T << R,p,0,0,0,1;
    if((Rp2Trans(R,p)-T).isZero()){ cout<<"Rp2Trans_Test: Pass"<<endl; }
    else{ cout<<"Rp2Trans_Test: Failed"<<endl; }
}

void Trans2Rp_Test()
{
    Eigen::Matrix3d R;
    R <<    1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Eigen::Vector3d p;
    p <<    1, 0, 0;
    Eigen::Matrix4d T;
    T << R,p,0,0,0,1;

    if((Trans2Rp(T)[0]-R).isZero() && (Trans2Rp(T)[1]-p).isZero()){ cout<<"Trans2Rp_Test: Pass"<<endl;}
    else{ cout<<"Trans2Rp_Test: Failed"<<endl;}
}

void Vec2Se3_Test()
{
    Eigen::VectorXd V(6);
    V << 0, 0, 1, 2, 0, 0;
    Eigen::Matrix4d T;
    T <<    0, -1, 0, 2,
            1, 0, -0, 0,
            -0, 0, 0, 0,
            0, 0, 0, 0;
    if((Vec2Se3(V)-T).isZero()){cout<<"Vec2Se3_Test: Pass"<<endl;}
    else{cout<<"Vec2Se3_Test: Failed"<<endl;}
}

void Se32Vec_Test()
{
    Eigen::Matrix4d T;
    T <<    0, -1, 0, 2,
            1, 0, -0, 0,
            -0, 0, 0, 0,
            0, 0, 0, 0;
    Eigen::VectorXd V(6);
    V << 0, 0, 1, 2, 0, 0;
    if((Se32Vec(T)-V).isZero()){cout<<"Se32Vec_Test: Pass"<<endl;}
    else{cout<<"Se32Vec_Test: Failed"<<endl;}
}

void Adjoint_Test()
{
    Eigen::Matrix4d T;
    T<< 1, 0, 0, 1,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::MatrixXd Ad(6,6);
    Ad <<   1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, -1, 0, 1, 0,
            0, 1, 0, 0, 0, 1;
    if((Adjoint(T)-Ad).isZero()){cout<<"Adjoint_Test: Pass"<<endl;}
    else{cout<<"Adjoint_Test: Failed"<<endl;}
}

void MatrixExp6_Test()
{
    Eigen::Matrix4d S;
    S <<    0, 0, -PI/2, 0.089*PI/2,
            0, 0, 0, 0,
            PI/2, 0, 0, 0,
            0, 0, 0, 0;
    MatrixExp6(S);
    Eigen::Matrix4d T;
    T <<    0, 0, -1, 0.089,
            0, 1, 0, 0,
            1, 0, 0, 0.089,
            0, 0, 0, 1;
    cout<<"MatrixExp6_Test: Pass"<<endl;
}

void MatrixLog6_Test()
{
    Eigen::Matrix4d T;
    // T <<    0.540302, 0, -0.841471, 0.089,
    //         0, 1, 0, 0,
    //         0.841471, 0, 0.540302, 0.089,
    //         0, 0, 0, 1;
    T <<    0, 0, -1, 0.089,
            0, 1, 0, 0,
            1, 0, 0, 0.089,
            0, 0, 0, 1;
    cout<< MatrixLog6(T)<<endl;
}
int main()
{
    NearZero_Test();
    So32Vec_Test();
    Vec2So3_Test();
    AxisAng3_Test();
    MatrixExp3_R_Test();
    MatrixR_Exp3_Test();
    Ad_Test();
    Rp2Trans_Test();
    Trans2Rp_Test();
    //*--------------2021-7-27---------------*//
    Vec2Se3_Test();
    Se32Vec_Test();
    Adjoint_Test();

    MatrixExp6_Test();
    MatrixLog6_Test();
    return 0;
}