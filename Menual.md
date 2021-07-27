# Screw Robotics Package Menual

[TOC]

# A

## Ad()

* **<font color='red'>Eigen::MatrixXd</font> Ad(<font color='green'>Eigen::VectorXd</font> <font color='Blue'>V</font>)**

  功能：通过给定的6维向量计算6*6矩阵[adV]

  Input: Eigen::VectorXd (6x1)

  Output: Eigen::MatrixXd (6x6)

  Note: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2

## Adjoint()

* **<font color='red'>Eigen::MatrixXd</font> Adjoint(<font color='green'>const Eigen::Matrix4d</font>& T)**

  Function: Provides the adjoint representation of a transformation matrix

  ​                 Used to change the frame of reference for spatial velocity vectors

  功能：提供一个转换矩阵的伴随表示，利用其可以改变空间速度向量的参考坐标系，例如 w_s = Ad{T_sb} * w_b

  Inputs: 4*4 Trasformation matrix SE3

  Returns: 6*6 Adjoint Representation fo the matrix

## AxisAng3()

* **<font color='red'>Eigen::Vector4d</font> AxisAng3(<font color='green'>const Eigen::Vector3d</font>& omega_theta)**

  Function: Translates an exponential rotation into

  ​                  it's individual components

  功能：返回一个omega_theta差分成旋转轴和旋转角

  Inputs: Exponential rotation(rotation matrix in terms of

  ​             a rotation axis and the angle of ratation)

  Returns: The axis and angle of rotation as [x, y, z, theta]



# M

## MatrixExp3_R()

* **<font color='red'>Eigen::Matrix3d</font> MatrixExp3(<font color='green'>const Eigen::Matrix3d</font>&)**

  Function: Translates an exponential rotation into a rotation matrix

  功能：将指数旋转变换为旋转矩阵

  Inputs: exponential representation of a rotation

  Returns: Rotation matrix

## MatrixR_Exp3()

* **<font color='red'>Eigen::Matrix3d</font> MatrixR_Exp3(<font color='green'>const Eigen::Matrix3d</font>& R)**

  Function: Computes the matrix logarithm of a rotation matrix

  功能： 计算一个旋转矩阵的指数旋转（逆解）

  Inputs: Rotation matrix

  Returns: exponential representation of a rotation

## MatrixExp6()

* **<font color='red'>Eigen::Matrix4d</font> MatrixExp6(<font color='green'>const Eigen::Matrix4d</font>& S)**

  Function: Rotation expanded for screw axis

  功能：输入旋量S转换至旋转矩阵

  Inputs: se3 matrix representation of exponential coordinates (transformation matrix)

  Returns: 4x4 Matrix representing the rotation

# N

## NearZero()

* **<font color='red'>bool</font> NearZero(<font color='green'>const double</font>)**

  Function: Find if the value is negligible enough to consider 0

  功能：查看输入数据是否可以替代为0

  Returns：Boolean of true-ignore or false-can't ignore

  返回值：布尔值，若可以则为true，否则为false



# R

## Rp2Trans()

* **<font color='red'>Eigen::Matrix4d</font> Rp2Trans(<font color='green'>const Eigen::Matrix3d</font>& R, <font color='green'>const Eigen::Vector3d</font>& p)**

  Function: Combines a rotation matrix and position vector into a single

  ​                  Special Euclidian Group(SE3) homogeneous transformation matrix

  功能：将旋转矩阵与位移向量合并为一个SE3的对称变换矩阵

  Inputs: Rotation Matrix (R), Position Vector (p)

  Returns: Matrix of T = [[R,p],

  ​                                      0,0,0,1]



# S

## So32Vec()

* **<font color='red'>Eigen::Vector3d</font> So32Vec(<font color='green'>const Eigen::Matrix3d</font>& matrix)**

  Function: Returns angular velocity vector represented by

  ​                  the screw symmetric matrix

  功能： 将旋量对角矩阵返回对应的角速度向量

  Input: Eigen::Matrix3d 3*3 screw symmetric matrix

  Output: Eigen::Vector3d 3*1 angular velocity vector

## Se32Vec()

* **<font color='red'>Eigen::VectorXd</font> Se32Vec(<font color='green'>const Eigen::Matrix4d</font>& T)**

  Function: Translates a transformation matrix into a spatial velocity vector

  功能：将变换矩阵转变为一个spatial速度向量

  Inputs: Transformation matrix

  Returns: Spatial velocity vector [angular velocity, linear velocity]



# T

## Trans2Rp()

* **<font color='red'>std::vector\<Eigen::MatrixXd\></font> Trans2Rp(<font color='green'>const Eigen::MatrixXd</font>& T)**

  Function: Separate the rotation matrix and position vector from

  ​                  the transfomation matrix representation

  功能：将从SE3的变换矩阵分离出旋转矩阵与位移向量

  Inputs: Homogeneous transformation matrix

  Returns: std::vector of rotationmatrix and position vector



# V

## Vec2So3()

* **<font color='red'>Eigen::Matrix3d</font> Vec2So3(<font color='green'>const Eigen::Vector3d</font>& omega)**

  Function: Returns the screw symmetric matrix representation of

  ​				  an angular velocity vector

  功能：通过输入的角速度向量返回对应旋量对称矩阵

  Input: Eigen::Vector3d 3*1 angular velocity vector

  Output: Eigen::Matrix3d 3*3 screw symmetric matrix

## Vec2Se3()

* **<font color='red'>Eigen::Matrix4d</font> Vec2Se3(<font color='green'>const Eigen::VectorXd</font>& V)**

  Function: Translates a spatial velocity vector into a transformation matrix

  功能：将一个spatial速度向量转换至转换矩阵

  Inputs: Spatial velocity vector [angular velocity, linear velocity]

  Returns: Transformation matrix