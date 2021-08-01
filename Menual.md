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

## AxisAng6()

* **<font color='red'>Eigen::VectorXd</font> AxisAng6(<font color='green'>const Eigen::VectorXd</font>& expc6)**

  Function: Translates a 6-vector of exponential coordinates into screw axis-angle form

  Inputs:

  ​	expc6: A 6-vector of exponential coordinates for rigid-body motion S*theta

  Returns: The corresponding normalized screw axis S; The distance theta traveled along/about S in form [S,theta]

# F

## FKinSpace()

* **<font color='red'>Eigen::Matrix4d</font> FKinSpace(<font color='green'>const Eigen::Matrix4d</font>& M, <font color='green'>const Eigen::MatrixXd</font>& Slist, <font color='green'>const Eigen::VectorXd</font>& thetalist)**

  Function: Compute end effector frame (used for current spatial position calculation)

  功能：计算末端执行器坐标系的姿态矩阵（正运动学）

  Inputs: Home configuration (position and orientation) of end-effector. The joint screw axes in the space frame when the manipulator is at the home position A list of joint coordinates.

  Returns: Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates

  Notes: FK means Forward Kinematics

## FKinBody()

* **<font color='red'>Eigen::Matrix4d</font> FKinBody(<font color='green'>const Eigen::Matrix4d</font>& M, <font color='green'>const Eigen::MatrixXd</font>& Blist, <font color='green'>const Eigen::VectorXd</font>& thetalist)**



# I

##     IKinSpace()

* **<font color='red'>bool IKinSpace</font>(<font color='green'>const Eigen::MatrixXd</font>& Slist, <font color='green'>const Eigen::MatrixXd</font>& M, <font color='green'>const Eigen::MatrixXd</font>& T,<font color='green'>Eigen::VectorXd</font>& thetalist, <font color='green'>double</font> eomg, <font color='green'>double</font> ev)**

  Function: Computes inverse kinematics in the space frame for an open chain robot

  Inputs:

  ​	Slist: The joint screw axis in the space frame when the manipulator is at the home position, in the format of a matrix with axis as the columns

  ​	M: The home cofiguration of the end-effector

  ​	T: The desired end-effector configuration Tsd

  ​	thetalist\[in][out]: An initial guess and result output of joint angles that are close to satisfying Tsd

  ​	emog: A small positive tolerance on the end-effector orientation error. The returned joint angles must give an end-effector orientation error less than eomg.

  ​	ev: A small positive tolerance on the end-effector linear position error. The returned joint angles must give an end-effector position error less than ev.

  Outputs:

  ​	success: A logical value where TRUE means that the function found a solution and FALSE means that it ran through the set number of maximum iterations without finding a solution within the tolerances eomg and ev.

  ​	thetalist\[in][out]: joint angles that achieve T within the specified tolerances.

##     IKinBody()

* **<font color='red'>bool IKinBody</font>(<font color='green'>const Eigen::MatrixXd</font>& Blist, <font color='green'>const Eigen::MatrixXd</font>& M, <font color='green'>const Eigen::MatrixXd</font>& T, <font color='green'>Eigen::VectorXd</font>& thetalist, <font color='green'>double</font> eomg, <font color='green'>double</font> ev)**

  Function: Computes inverse kinematics in the body frame for an open chain robot

  Inputs:

  ​	Blist: The joint screw axes in the end-effector frame when the manipulator is at the home position, in the format of a matrix with axes as the columns

  ​	M: The home configuration of the end-effector

  ​	T: The desired end-effector configuration Tsd

  ​	thetalist\[in][out]: An initial guess and result output of joint angles that are close to satisfying Tsd

  ​	emog: A small positive tolerance on the end-effector orientation error. The returned joint angles must give an end-effector orientation error less than eomg

  ​	ev: A small positive tolerance on the end-effector linear position error. The returned joint angles must give an end-effector position error less than ev

  Outputs:

  ​	success: A logical value where TRUE means that the function found a solution and FALSE means that it ran through the set number of maximum iterations without finding a solution within the tolerances eomg and ev.

  ​	thetalist\[in][out]: Joint angles that achieve T within the specified tolerances,

# J

##  JacobianSpace()

* **<font color='red'>Eigen::MatrixXd</font> JacobianSpace(<font color='green'>const Eigen::MatrixXd</font>& Slist, <font color='green'>const Eigen::MatrixXd</font>& thetalist)**

  Function: Gives the space Jacobian

  功能：获得一个空间坐标系下的雅克比矩阵

  Inputs: Screw axis in home position, joint configuration

  Returns: 6xn Spatial Jacobian

  Page: 180 & Page: 100 Definition 3.20

## JacobianBody()

* **<font color='red'>Eigen::MatrixXd</font> JacobianBody(<font color='green'>const Eigen::MatrixXd</font>& Blist, <font color='green'>const Eigen::MatrixXd</font>& thetalist)**

  Function: Gives the body Jacobian

  功能：获得一个物体坐标系下的雅克比矩阵

  Inputs: Screw axis in BODY position, joint configuration

  Returns: 6xn Bobdy Jacobian

  Page: 185 & Page: 100 Definition 3.20

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

## RotInv()

* **<font color='red'>Eigen::MatrixXd</font> RotInv(<font color='green'>const Eigen::MatrixXd</font>& rotMatrix)**

  Function: Inverts a rotation matrix

  Inputs: A rotation matrix R

  Returns: The inverse of R

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

## Screw2Axis()

* **<font color='red'>Eigen::VectorXd</font> Screw2Axis(<font color='green'>Eigen::Vector3d</font> q, <font color='green'>Eigen::Vector3d</font> s, <font color='green'>double</font> h)**

  Function: Takes a parametric description of a screw axis and converts

  ​		           it to a normalized screw axis

  Inputs:

  ​	q: A point lying on the screw axis

  ​	s: A unit vector in the direction of the screw axis

  ​	h: The pitch of the screw axis

  Returns: A normalized screw axis described by the input

# T

## Trans2Rp()

* **<font color='red'>std::vector\<Eigen::MatrixXd\></font> Trans2Rp(<font color='green'>const Eigen::MatrixXd</font>& T)**

  Function: Separate the rotation matrix and position vector from

  ​                  the transfomation matrix representation

  功能：将从SE3的变换矩阵分离出旋转矩阵与位移向量

  Inputs: Homogeneous transformation matrix

  Returns: std::vector of rotationmatrix and position vector

## TransInv()

* **<font color='red'>Eigen::Matrix4d</font> TransInv(<font color='green'>const Eigen::Matrix4d</font>& transform)**

  Function: Inverts a homogeneous transformation matrix

  Inputs: A homogeneous transformation Matrix T

  Returns: The inverse of T

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