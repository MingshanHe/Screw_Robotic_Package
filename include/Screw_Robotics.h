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

    /*
	 * Function: Calculate the 6x6 matrix [adV] of the given 6-vector
     * 功能：通过给定的6维向量计算6*6矩阵[adV]
	 * Input: Eigen::VectorXd (6x1)
	 * Output: Eigen::MatrixXd (6x6)
	 * Note: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2
	 */
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

    /*
     * Function: Translates a spatial velocity vector into a transformation matrix
     * 功能：将一个spatial速度向量转换至转换矩阵
     * Inputs: Spatial velocity vector [angular velocity, linear velocity]
     * Returns: Transformation matrix
    */
    Eigen::Matrix4d Vec2Se3(const Eigen::VectorXd&);

    /*
     * Function: Translates a transformation matrix into a spatial velocity vector
     * 功能：将变换矩阵转变为一个spatial速度向量
     * Inputs: Transformation matrix
     * Returns: Spatial velocity vector [angular velocity, linear velocity]
    */
    Eigen::VectorXd Se32Vec(const Eigen::Matrix4d&);

    /*
     * Function: Provides the adjoint representation of a transformation matrix
     *              Used to change the frame of reference for spatial velocity vectors
     * 功能：提供一个转换矩阵的伴随表示，利用其可以改变空间速度向量的参考坐标系，例如 w_s = Ad{T_sb} * w_b
     * Inputs: 4*4 Trasformation matrix SE3
     * Returns: 6*6 Adjoint Representation fo the matrix
    */
    //* Page: 100
    Eigen::MatrixXd Adjoint(const Eigen::Matrix4d&);

	/*
     * Function: Rotation expanded for screw axis
     * 功能：输入旋量S转换至旋转矩阵
	 * Inputs: se3 matrix representation of exponential coordinates (transformation matrix)
	 * Returns: 4x4 Matrix representing the rotation
	 */
	Eigen::Matrix4d MatrixExp6(const Eigen::Matrix4d&);

	/*
     * Function: Rotation transform to screw axis
     * 功能：输入旋量矩阵转换至旋转S向量
	 * Inputs: 4x4 Matrix representing the rotation
	 * Returns: se3 matrix representation of exponential coordinates (transformation matrix)
	 */
    Eigen::Matrix4d MatrixLog6(const Eigen::Matrix4d&);

    /*
     * Function: Compute end effector frame (used for current spatial position calculation)
     * 功能：计算末端执行器坐标系的姿态矩阵（正运动学）
	 * Inputs: Home configuration (position and orientation) of end-effector
	 *		   The joint screw axes in the space frame when the manipulator
	 *             is at the home position
	 * 		   A list of joint coordinates.
	 * Returns: Transfomation matrix representing the end-effector frame when the joints are
	 *				at the specified coordinates
	 * Notes: FK means Forward Kinematics
	 */
    //* Page: 148
    Eigen::Matrix4d FKinSpace(
        const Eigen::Matrix4d& ,
        const Eigen::MatrixXd& ,
        const Eigen::VectorXd& );
    //* Page: 152
    Eigen::Matrix4d FKinBody(
        const Eigen::Matrix4d& ,
        const Eigen::MatrixXd& ,
        const Eigen::VectorXd& );

    /*
     * Function: Gives the space Jacobian
     * 功能：获得一个空间坐标系下的雅克比矩阵
	 * Inputs: Screw axis in home position, joint configuration
	 * Returns: 6xn Spatial Jacobian
	 */
    //* Page: 180 & Page: 100 Definition 3.20
    Eigen::MatrixXd JacobianSpace(
        const Eigen::MatrixXd& ,
        const Eigen::MatrixXd& );
	/*
	 * Function: Gives the body Jacobian
     * 功能：获得一个物体坐标系下的雅克比矩阵
	 * Inputs: Screw axis in BODY position, joint configuration
	 * Returns: 6xn Bobdy Jacobian
	 */
    //* Page: 185 & Page: 100 Definition 3.20
    Eigen::MatrixXd JacobianBody(
        const Eigen::MatrixXd& ,
        const Eigen::MatrixXd& );
    /*
     * Function: Inverts a homogeneous transformation matrix
     * Inputs: A homogeneous transformation Matrix T
     * Returns: The inverse of T
    */
    Eigen::Matrix4d TransInv(const Eigen::Matrix4d&);
    /*
     * Function: Inverts a rotation matrix
     * Inputs: A rotation matrix R
     * Returns: The inverse of R
    */
    Eigen::MatrixXd RotInv(const Eigen::MatrixXd&);
    /*
     * Function: Takes a parametric description of a screw axis and converts
     *           it to a normalized screw axis
     * Inputs:
     * q: A point lying on the screw axis
     * s: A unit vector in the direction of the screw axis
     * h: The pitch of the screw axis
     * Returns: A normalized screw axis described by the input
    */
    Eigen::VectorXd Screw2Axis(Eigen::Vector3d q, Eigen::Vector3d s, double h);
    /*
     * Function: Translates a 6-vector of exponential coordinates into screw axis-angle form
     * Inputs:
     * expc6: A 6-vector of exponential coordinates for rigid-body motion S*theta
     * Returns: The corresponding normalized screw axis S; The distance theta traveled along/about S in form [S,theta]
    */
    Eigen::VectorXd AxisAng6(const Eigen::VectorXd&);
    /*
     * Function: Computes inverse kinematics in the space frame for an open chain robot
     * Inputs:
     * Slist: The joint screw axis in the space frame when the manipulator is at the home position,
     *          in the format of a matrix with axis as the columns
     * M: The home cofiguration of the end-effector
     * T: The desired end-effector configuration Tsd
     * thetalist[in][out]: An initial guess and result output of joint angles that are close to satisfying Tsd
     * emog: A small positive tolerance on the end-effector orientation error.
     *          The returned joint angles must give an end-effector orientation error less than eomg.
     * ev: A small positive tolerance on the end-effector linear position error.
     *          The returned joint angles must give an end-effector position error less than ev.
     * Outputs:
     * success: A logical value where TRUE means that the function found a solution
     *          and FALSE means that it ran through the set number of maximum iterations without
     *          finding a solution within the tolerances eomg and ev.
     * thetalist[in][out]: joint angles that achieve T within the specified tolerances.
    */
    bool IKinSpace(
        const Eigen::MatrixXd& ,
        const Eigen::MatrixXd& ,
        const Eigen::MatrixXd& ,
        Eigen::VectorXd& ,
        double,
        double);
}
#endif