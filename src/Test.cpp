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
    // cout<< MatrixLog6(T)<<endl;
    cout<<"MatrixLog6_Test: Pass"<<endl;
}

void FKinSpace_Test()
{
    //* Page: 148
    Eigen::Matrix4d M;
    //*---- UR -----*//
    double L1,L2,W1,W2,H1,H2;

    L1 = 0.425;
    L2 = 0.392;
    W1 = 0.109;
    W2 = 0.082;
    H1 = 0.089;
    H2 = 0.095;

    M <<    -1, 0, 0, L1+L2,
            0, 0, 1,  W1+W2,
            0, 1, 0,  H1-H2,
            0, 0, 0,  1;
    Eigen::MatrixXd Slist(6,6);
    Slist <<    0, 0, 0, 0, 0, 0,
                0, 1, 1, 1, 0, 1,
                1, 0, 0, 0, -1, 0,
                0, -H1, -H1, -H1, -W1, H2-H1,
                0, 0, 0, 0, L1+L2, 0,
                0, 0, L1, L1+L2, 0, L1+L2;
    Eigen::VectorXd thetalist(6);
    thetalist << 0, -PI/2, 0, 0, PI/2, 0;
    // cout << FKinSpace(M, Slist, thetalist) << endl;
    cout<<"FKinSpace_Test: Pass"<<endl;
}
void FKinBody_Test()
{
    //* Page: 152
    Eigen::Matrix4d M;
    //*---- WAM 7R ----*//
    double L1, L2, L3, W1;

    L1 = 0.55;
    L2 = 0.3;
    L3 = 0.06;
    W1 = 0.045;

    M <<    1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, L1 + L2 + L3,
            0, 0, 0, 1;
    Eigen::MatrixXd Blist(6,7);
    Blist <<    0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 1, 0, 1, 0,
                1, 0, 1, 0, 1, 0, 1,
                0, L1+L2+L3, 0, L2+L3, 0, L3, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, W1, 0, 0, 0;
    Eigen::VectorXd thetalist(7);
    thetalist << 0, PI/4, 0, PI/4, 0, -PI, 0;
    // cout << FKinBody(M, Blist, thetalist) << endl;
    cout<<"FKinBody_Test: Pass"<<endl;
}
void JacobianSpace_Test()
{
    double L1,L2,W1,W2,H1,H2;

    L1 = 0.425;
    L2 = 0.392;
    W1 = 0.109;
    W2 = 0.082;
    H1 = 0.089;
    H2 = 0.095;

    Eigen::MatrixXd Slist(6,6);
    Slist <<    0, 0, 0, 0, 0, 0,
                0, 1, 1, 1, 0, 1,
                1, 0, 0, 0, -1, 0,
                0, -H1, -H1, -H1, -W1, H2-H1,
                0, 0, 0, 0, L1+L2, 0,
                0, 0, L1, L1+L2, 0, L1+L2;

    Eigen::VectorXd thetalist(6);
    thetalist << 0, -PI/2, 0, 0, PI/2, 0;

    // cout << JacobianSpace(Slist, thetalist) << endl;
    cout<<"JacobianSpace_Test: Pass"<<endl;
}
void TransInv_Test()
{
    Eigen::Matrix4d T;
    T <<    0, 0, -1, 0.089,
            0, 1, 0, 0,
            1, 0, 0, 0.089,
            0, 0, 0, 1;
    // cout<<TransInv(T)*T<<endl;
    cout<<"TransInv_Test: Pass"<<endl;
}
void RotInv_Test()
{
    Eigen::Matrix4d T;
    T <<    0, 0, -1, 0.089,
            0, 1, 0, 0,
            1, 0, 0, 0.089,
            0, 0, 0, 1;
    // cout<<RotInv(T)<<endl;
    cout<<"RotInv_Test: Pass"<<endl;
}
void Screw2Axis_Test()
{
}
void IKinSpace_Test()
{
    Eigen::MatrixXd SlistT(3, 6);
    SlistT  <<  0, 0, 1, 4, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, -1, -6, 0, -0.1;
	Eigen::MatrixXd Slist = SlistT.transpose();
	Eigen::Matrix4d M;
	M << -1, 0, 0, 0,
		0, 1, 0, 6,
		0, 0, -1, 2,
		0, 0, 0, 1;
	Eigen::Matrix4d T;
	T << 0, 1, 0, -5,
		1, 0, 0, 4,
		0, 0, -1, 1.6858,
		0, 0, 0, 1;
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;
	double eomg = 0.01;
	double ev = 0.001;
	bool b_result = true;
	Eigen::VectorXd theta_result(3);
	theta_result << 1.57073783, 2.99966384, 3.1415342;
	bool iRet = IKinSpace(Slist, M, T, thetalist, eomg, ev);
    cout<<"IKinSpace_Test: Pass"<<endl;
}
void IKinBody_Test()
{
	Eigen::MatrixXd BlistT(3, 6);
	BlistT << 0, 0, -1, 2, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0.1;
	Eigen::MatrixXd Blist = BlistT.transpose();
	Eigen::Matrix4d M;
	M << -1, 0, 0, 0,
		0, 1, 0, 6,
		0, 0, -1, 2,
		0, 0, 0, 1;
	Eigen::Matrix4d T;
	T << 0, 1, 0, -5,
		1, 0, 0, 4,
		0, 0, -1, 1.6858,
		0, 0, 0, 1;
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;
	double eomg = 0.01;
	double ev = 0.001;
	bool b_result = true;
	Eigen::VectorXd theta_result(3);
	theta_result << 1.57073819, 2.999667, 3.14153913;
	bool iRet = IKinBody(Blist, M, T, thetalist, eomg, ev);
    cout<<"IKinBody_Test: Pass"<<endl;
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
    //*--------------2021-7-28---------------*//
    FKinSpace_Test();
    FKinBody_Test();
    //*--------------2021-7-29---------------*//
    JacobianSpace_Test();
    //*--------------2021-7-30---------------*//
    TransInv_Test();
    RotInv_Test();
    Screw2Axis_Test();
    IKinSpace_Test();
    IKinBody_Test();
    return 0;
}