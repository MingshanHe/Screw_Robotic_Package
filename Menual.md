# Screw Robotics Package Menual

[TOC]

# A

## AxisAng3()

* **Eigen::Vector4d AxisAng3(const Eigen::Vector3d& omega_theta)**

  Function: Translates an exponential rotation into

  ​                  it's individual components

  功能：返回一个omega_theta差分成旋转轴和旋转角

  Inputs: Exponential rotation(rotation matrix in terms of

  ​             a rotation axis and the angle of ratation)

  Returns: The axis and angle of rotation as [x, y, z, theta]



# M

## MatrixExp3_R()

* **Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d&)**

  Function: Translates an exponential rotation into a rotation matrix

  功能：将指数旋转变换为旋转矩阵

  Inputs: exponential representation of a rotation

  Returns: Rotation matrix

## MatrixR_Exp3()

* **Eigen::Matrix3d MatrixR_Exp3(const Eigen::Matrix3d& R)**

  Function: Computes the matrix logarithm of a rotation matrix

  功能： 计算一个旋转矩阵的指数旋转（逆解）

  Inputs: Rotation matrix

  Returns: exponential representation of a rotation

# N

## NearZero()

* **bool NearZero(const double)**

  Function: Find if the value is negligible enough to consider 0

  功能：查看输入数据是否可以替代为0

  Returns：Boolean of true-ignore or false-can't ignore

  返回值：布尔值，若可以则为true，否则为false





# S

## So32Vec()

* **Eigen::Vector3d So32Vec(const Eigen::Matrix3d& matrix)**

  Function: Returns angular velocity vector represented by

  ​                  the screw symmetric matrix

  功能： 将旋量对角矩阵返回对应的角速度向量

  Input: Eigen::Matrix3d 3*3 screw symmetric matrix

  Output: Eigen::Vector3d 3*1 angular velocity vector





# V

## Vec2So3()

* **Eigen::Matrix3d Vec2So3(const Eigen::Vector3d& omega)**

  Function: Returns the screw symmetric matrix representation of

  ​				  an angular velocity vector

  功能：通过输入的角速度向量返回对应旋量对称矩阵

  Input: Eigen::Vector3d 3*1 angular velocity vector

  Output: Eigen::Matrix3d 3*3 screw symmetric matrix