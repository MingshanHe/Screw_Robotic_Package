#ifndef SCREW_ROBOTICS_H
#define SCREW_ROBOTICS_H
#pragma once

#include <Eigen/Core>
#include <vector>
#include <iostream>

namespace screw_robotics{
    /*
     * Function: Find if the value is negligible enough to consider 0
     * 功能：查看输入数据是否可以替代为0
     * Returns：Boolean of true-ignore or false-can't ignore
     * 返回值：布尔值，若可以则为true，否则为false
    */
    bool NearZero(const double);

    /*
     * Function: Returns the screw symmetric matrix representation of
     *           an angular velocity vector
     * 功能：通过输入的角速度向量返回对应旋量对称矩阵
     * Input: Eigen::Vector3d 3*1 angular velocity vector
     * Output: Eigen::Matrix3d 3*3 screw symmetric matrix
    */
    Eigen::Matrix3d Vec2So3(const Eigen::Vector3d& omega);

    /*
     * Function: Returns angular velocity vector represented by
     *           the screw symmetric matrix
     * 功能： 将旋量对角矩阵返回对应的角速度向量
     * Input: Eigen::Matrix3d 3*3 screw symmetric matrix
     * Output: Eigen::Vector3d 3*1 angular velocity vector
    */
    Eigen::Vector3d So32Vec(const Eigen::Matrix3d& matrix);
}
#endif