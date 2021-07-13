#include "../include/Screw_Robotics.h"

namespace screw_robotics{

    //* Function: Find if the value is negligible enough to consider 0
    bool NearZero(const double val)
    {
        return (std::abs(val)<0.00001);
    }

    //* Function: Returns the screw symmetric matrix representation of
    //*           an angular velocity vector
    Eigen::Matrix3d Vec2So3(const Eigen::Vector3d& omega_)
    {
        Eigen::Matrix3d omega_matrix_;
        omega_matrix_ << 0, -omega_(2), omega_(1),
                        omega_(2), 0, -omega_(0),
                        -omega_(1), omega_(0), 0;
        return omega_matrix_;
    }

    //* Function: Returns angular velocity vector represented by
    //*           the screw symmetric matrix
    Eigen::Vector3d So32Vec(const Eigen::Matrix3d& omega_matrix_)
    {
        Eigen::Vector3d omega_;
        omega_ << omega_matrix_(2,1), omega_matrix_(0,2), omega_matrix_(1,0);
        return omega_;
    }
}