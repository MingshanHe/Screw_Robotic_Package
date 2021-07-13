#include "../include/Screw_Robotics.h"
using namespace std;

int main()
{
    Eigen::Vector3d omega;
    Eigen::Matrix3d omega_matrix;
    omega << 0, 1, 0;
    omega_matrix = screw_robotics::Vec2So3(omega);
    cout << omega_matrix << endl;
    // omega_matrix << 1,0,0,0,1,0,0,0,1;
    // omega = screw_robotics::So32Vec(omega_matrix);
    cout<<screw_robotics::So32Vec(omega_matrix)<<endl;
    return 0;
}