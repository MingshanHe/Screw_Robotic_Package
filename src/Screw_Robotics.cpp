#include "../include/Screw_Robotics.h"
#include <Eigen/Dense>
#define PI 3.1415926

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


    //* Function: Returns a normalized version of the input vector
    Eigen::MatrixXd Normalize(Eigen::MatrixXd V)
    {
        V.normalize();
        return V;
    }

    //* Function: Translates an exponential rotation into
    //*            it's individual components
    Eigen::Vector4d AxisAng3(const Eigen::Vector3d& omega_theta)
    {
        Eigen::Vector4d v_ret;
        v_ret << Normalize(omega_theta), omega_theta.norm();
        return v_ret;
    }

    //* Function: Translates an exponential rotation into a rotation matrix
    Eigen::Matrix3d MatrixExp3_R(const Eigen::Matrix3d& so3mat)
    {
        Eigen::Vector3d omega_theta = So32Vec(so3mat);
        Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
        if (NearZero(so3mat.norm())){
            return m_ret;
        }
        else
        {
            double theta = (AxisAng3(omega_theta))(3);
            Eigen::Matrix3d omega_mat = so3mat * (1/theta);
            return m_ret + std::sin(theta) * omega_mat + ((1-std::cos(theta))*(omega_mat * omega_mat));
        }
    }

    //* Function: Computes the matrix logarithm of a rotation matrix
    Eigen::Matrix3d MatrixR_Exp3(const Eigen::Matrix3d& R)
    {
        double acosinput = (R.trace() - 1) / 2.0;
        Eigen::Matrix3d m_ret;
        m_ret.setZero();
        if(acosinput >= 1)
        {
            return m_ret;
        }
        else if(acosinput <= -1)
        {
            Eigen::Vector3d omega;
            if(!NearZero(1+R(2,2))){ omega = (1.0/std::sqrt(2*(1+R(2,2))))*Eigen::Vector3d(R(0,2), R(1,2), R(2,2)); }
            else if(!NearZero(1+R(1,1))){ omega = (1.0/std::sqrt(2*(1+R(1,1))))*Eigen::Vector3d(R(0,1), R(1,1), R(2,1)); }
            else { omega = (1.0/std::sqrt(2*(1+R(0,0))))*Eigen::Vector3d(R(0,0), R(1,0), R(2,0)); }
            m_ret = Vec2So3(PI * omega);
            return m_ret;
        }
        else
        {
            double theta = std::acos(acosinput);
            m_ret = theta/ 2.0 / sin(theta)*(R-R.transpose());
            return m_ret;
        }
    }
    //*  Function: Calculate the 6x6 matrix [adV] of the given 6-vector
    Eigen::MatrixXd Ad(Eigen::VectorXd V)
    {
        Eigen::Matrix3d omega_mat = Vec2So3(Eigen::Vector3d(V(0), V(1), V(2)));

        Eigen::MatrixXd result(6,6);
        result.topLeftCorner<3,3>() = omega_mat;
        result.topRightCorner<3,3>() = Eigen::Matrix3d::Zero(3,3);
        result.bottomLeftCorner<3,3>() = Vec2So3(Eigen::Vector3d(V(3), V(4), V(5)));
        result.bottomRightCorner<3,3>() = omega_mat;
        return result;
    }

    //* Function: Combines a rotation matrix and position vector into a single
    //*           Special Euclidian Group(SE3) homogeneous transformation matrix
    Eigen::Matrix4d Rp2Trans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
    {
        Eigen::Matrix4d T;
        T <<    R,p,
                0,0,0,1;
        return T;
    }
    //* Function: Separate the rotation matrix and position vector from
    //*           the transfomation matrix representation
    std::vector<Eigen::MatrixXd> Trans2Rp(const Eigen::MatrixXd& T)
    {
        std::vector<Eigen::MatrixXd> Rp_ret;
        Eigen::Matrix3d R_ret;
        R_ret = T.block<3,3>(0,0);

        Eigen::Vector3d p_ret(T(0,3), T(1,3), T(2,3));

        Rp_ret.push_back(R_ret);
        Rp_ret.push_back(p_ret);
        return Rp_ret;
    }
    //* Function: Translates a spatial velocity vector into a transformation matrix
    Eigen::Matrix4d Vec2Se3(const Eigen::VectorXd& V)
    {
        Eigen::Vector3d exp(V(0), V(1), V(2));
        Eigen::Vector3d linear(V(3), V(4), V(5));

        Eigen::Matrix4d m_ret;
        m_ret <<    Vec2So3(exp), linear,
                    0,0,0,0;

        return m_ret;
    }
    //* Function: Translates a transformation matrix into a spatial velocity vector
    Eigen::VectorXd Se32Vec(const Eigen::Matrix4d& T)
    {
        Eigen::VectorXd V(6);
        V << T(2,1), T(0,2), T(1,0), T(0,3), T(1,3), T(2,3);

        return V;
    }
    //* Function: Provides the adjoint representation of a transformation matrix
    //*             Used to change the frame of reference for spatial velocity vectors
    Eigen::MatrixXd Adjoint(const Eigen::Matrix4d& T)
    {
        std::vector<Eigen::MatrixXd> Rp = Trans2Rp(T);
        Eigen::MatrixXd ad_ret(6,6);
        Eigen::MatrixXd Zero = Eigen::MatrixXd::Zero(3,3);
        ad_ret <<   Rp[0], Zero,
                    Vec2So3(Rp[1])*Rp[0], Rp[0];

        return ad_ret;
    }
    //* Function: Rotation expanded for screw axis
    Eigen::Matrix4d MatrixExp6(const Eigen::Matrix4d& S)
    {
        Eigen::Matrix3d omg_mat   = S.block<3,3>(0,0);
        Eigen::Vector3d omgtheta  = So32Vec(omg_mat);

        Eigen::MatrixXd m_ret(4,4);

        if(NearZero(omgtheta.norm()))
        {
            // std::cout<<"1....."<<std::endl;
            omg_mat = Eigen::MatrixXd::Identity(3,3);
            omgtheta << S(0,3), S(1,3), S(2,3);
            m_ret<< omg_mat, omgtheta,
                    0, 0, 0, 1;
            return m_ret;
        }
        else
        {
            // std::cout<<"2....."<<std::endl;
            double theta = (AxisAng3(omgtheta))(3);
            Eigen::Matrix3d omg_mat = S.block<3,3>(0,0) / theta;

            Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3,3) * theta + (1- std::cos(theta)) * omg_mat + ((theta - std::sin(theta)) * (omg_mat * omg_mat));
            Eigen::Vector3d linear(S(0,3), S(1,3), S(2,3));
            Eigen::Vector3d GThetaV = (expExpand * linear) / theta;
            // m_ret << MatrixExp3_R(omg_mat), GThetaV,
            //             0, 0, 0, 1;
            m_ret << MatrixExp3_R(omg_mat * theta), GThetaV,
                        0, 0, 0, 1;
            return m_ret;
        }
    }
    //* Function: Rotation transform to screw axis
    Eigen::Matrix4d MatrixLog6(const Eigen::Matrix4d& T)
    {
        Eigen::Matrix4d S;
        auto Rp = screw_robotics::Trans2Rp(T);
        Eigen::Matrix3d omgmat = screw_robotics::MatrixR_Exp3(Rp[0]);
        Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3,3);

        if (NearZero(omgmat.norm()))
        {
            // std::cout<<"1....."<<std::endl;
            S << zeros3d, Rp[1],
                    0, 0, 0, 0;
        }
        else
        {
            // std::cout<<"2....."<<std::endl;
            double theta = std::acos((Rp[0].trace() - 1) / 2.0);
            Eigen::Matrix3d logExpand  = (Eigen::MatrixXd::Identity(3,3) * theta + (1- std::cos(theta)) * omgmat / theta + ((theta - std::sin(theta)) * ((omgmat/theta) * (omgmat/theta)))).inverse();
            S << omgmat, logExpand*(Rp[1]*theta),
                    0, 0, 0, 0;
        }
        return S;
    }
    //* Function: Compute end effector frame (used for current spatial position calculation)
    Eigen::Matrix4d FKinSpace(const Eigen::Matrix4d& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetalist)
    {
        Eigen::Matrix4d T = M;
        for (int i = (thetalist.size() - 1); i > -1; i--)
        {
            T = MatrixExp6(Vec2Se3(Slist.col(i)*thetalist(i)))*T;
        }
        return T;
    }
    //* Function: Compute end effector frame (used for current spatial position calculation)
    Eigen::Matrix4d FKinBody(const Eigen::Matrix4d& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetalist)
    {
        Eigen::Matrix4d T = M;
        for (int i = 0; i < thetalist.size(); i++)
        {
            T = T * MatrixExp6(Vec2Se3(Blist.col(i)*thetalist(i)));
        }
        return T;
    }

    //* Function: Gives the space Jacobian
    Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetalist)
    {
        Eigen::MatrixXd Js = Slist;
        Eigen::MatrixXd T  = Eigen::MatrixXd::Identity(4,4);
        Eigen::VectorXd sListTemp(Slist.col(0).size());
        for (int i = 1; i < thetalist.size(); i++)
        {
            sListTemp << Slist.col(i - 1) * thetalist(i - 1);
            T = T * MatrixExp6(Vec2Se3(sListTemp));
            Js.col(i) = Adjoint(T) * Slist.col(i);
        }
        return Js;
    }
    //* Function: Gives the body Jacobian
    Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& thetalist)
    {
        Eigen::MatrixXd Jb = Blist;
        Eigen::MatrixXd T  = Eigen::MatrixXd::Identity(4,4);
        Eigen::VectorXd bListTemp(Blist.col(0).size());
        for (int i = thetalist.size() - 2; i >= 0; i--)
        {
            bListTemp << Blist.col(i + 1) * thetalist(i + 1);
            T = T * MatrixExp6(Vec2Se3(-1 * bListTemp));
            Jb.col(i) = Adjoint(T) * Blist.col(i);
        }
        return Jb;
    }
    //* Function: Inverts a homogeneous transformation matrix
    Eigen::Matrix4d TransInv(const Eigen::Matrix4d& transform)
    {
        auto Rp = Trans2Rp(transform);
        auto Rt = Rp[0].transpose();
        auto t  = -(Rt * Rp[1]);
        Eigen::MatrixXd inv(4,4);
        inv = Eigen::MatrixXd::Zero(4,4);
        inv.block(0, 0, 3, 3) = Rt;
        inv.block(0, 3, 3, 1) = t;
        inv(3,3) = 1;
        return inv;
    }
    //* Function: Inverts a rotation matrix
    Eigen::MatrixXd RotInv(const Eigen::MatrixXd& rotMatrix)
    {
        return rotMatrix.transpose();
    }
    //* Function: Takes a parametric description of a screw axis and converts
    //*           it to a normalized screw axis
    Eigen::VectorXd Screw2Axis(Eigen::Vector3d q, Eigen::Vector3d s, double h)
    {
        Eigen::VectorXd axis(6);
        axis.segment(0, 3) = s;
        axis.segment(3, 3) = q.cross(s) + (h * s);
        return axis;
    }
    //* Function: Translates a 6-vector of exponential coordinates into screw axis-angle form
    Eigen::VectorXd AxisAng6(const Eigen::VectorXd& expc6)
    {
        Eigen::VectorXd v_ret(7);
        double theta = Eigen::Vector3d(expc6(0), expc6(1), expc6(2)).norm();
        if(NearZero(theta))
        {
            theta = Eigen::Vector3d(expc6(3), expc6(4), expc6(5)).norm();
        }
        v_ret << expc6 / theta, theta;
        return v_ret;
    }

    bool IKinSpace(
        const Eigen::MatrixXd& Slist,
        const Eigen::MatrixXd& M,
        const Eigen::MatrixXd& T,
        Eigen::VectorXd& thetalist,
        double eomg,
        double ev)
    {
        int i = 0;
        int maxiterations = 20;
        Eigen::MatrixXd Tfk     = FKinSpace(M, Slist, thetalist);
        Eigen::MatrixXd Tdiff   = TransInv(Tfk)*T;
        Eigen::VectorXd Vs      = Adjoint(Tfk)*Se32Vec(MatrixLog6(Tdiff));
        Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
        Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

        bool err = (angular.norm() > eomg || linear.norm() > ev);
        Eigen::MatrixXd Js;
        while(err && i < maxiterations)
        {
            Js = JacobianSpace(Slist, thetalist);
            thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
            i += 1;
            Tfk     = FKinSpace(M, Slist, thetalist);
            Tdiff   = TransInv(Tfk) * T;
            Vs      = Adjoint(Tfk)*Se32Vec(MatrixLog6(Tdiff));
            angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
            linear  = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
            err = (angular.norm()>eomg || linear.norm() > ev);
        }
        std::cout<< thetalist << std::endl;
        return !err;
    }
}