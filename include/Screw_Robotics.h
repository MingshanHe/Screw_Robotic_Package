#ifndef SCREW_ROBOTICS_H
#define SCREW_ROBOTICS_H
#pragma once

#include <Eigen/Core>
#include <vector>
#include <iostream>

#define PI 3.1415926

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
    Eigen::Matrix3d Vec2So3(const Eigen::Vector3d&);

    /*
     * Function: Returns angular velocity vector represented by
     *           the screw symmetric matrix
     * 功能： 将旋量对角矩阵返回对应的角速度向量
     * Input: Eigen::Matrix3d 3*3 screw symmetric matrix
     * Output: Eigen::Vector3d 3*1 angular velocity vector
    */
    Eigen::Vector3d So32Vec(const Eigen::Matrix3d&);

    /*
     * Function: Returns a normalized version of the input vector
     * 功能：返回一个输入向量的规范单一化
     * Note: MatrixXd is used instaead of VectorXd for the case of
     *       row vectors
    */
    Eigen::MatrixXd Normalize(Eigen::MatrixXd);

    /*
     * Function: Translates an exponential rotation into
     *           it's individual components
     * 功能：返回一个omega_theta差分成旋转轴和旋转角
     * Inputs: Exponential rotation(rotation matrix in terms of
     *         a rotation axis and the angle of ratation)
     * Returns: The axis and angle of rotation as [x, y, z, theta]
    */
    Eigen::Vector4d AxisAng3(const Eigen::Vector3d&);

    /*
     * Function: Translates an exponential rotation into a rotation matrix
     * 功能：将指数旋转变换为旋转矩阵（正解）
     * Inputs: exponential representation of a rotation
     * Returns: Rotation matrix
    */
    Eigen::Matrix3d MatrixExp3_R(const Eigen::Matrix3d&);

    /*
     * Function: Computes the matrix logarithm of a rotation matrix
     * 功能： 计算一个旋转矩阵的指数旋转（逆解）
     * Inputs: Rotation matrix
     * Returns: exponential representation of a rotation
    */
    Eigen::Matrix3d MatrixR_Exp3(const Eigen::Matrix3d&);

    //TODO: Description
    Eigen::MatrixXd Ad(Eigen::VectorXd);

    /*
     * Function: Combines a rotation matrix and position vector into a single
     *           Special Euclidian Group(SE3) homogeneous transformation matrix
     * 功能：将旋转矩阵与位移向量合并为一个SE3的对称变换矩阵
     * Inputs: Rotation Matrix (R), Position Vector (p)
     * Returns: Matrix of T = [[R,p],
     *                          0,0,0,1]
    */
    Eigen::Matrix4d Rp2Trans(const Eigen::Matrix3d&, const Eigen::Vector3d&);
    /*
     * Function: Separate the rotation matrix and position vector from
     *           the transfomation matrix representation
     * 功能：将从SE3的变换矩阵分离出旋转矩阵与位移向量
     * Inputs: Homogeneous transformation matrix
     * Returns: std::vector of rotationmatrix and position vector
    */
    std::vector<Eigen::MatrixXd> Trans2Rp(const Eigen::MatrixXd&);
}
#endif