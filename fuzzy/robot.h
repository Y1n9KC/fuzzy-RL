#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <math.h>

using namespace std;

class Aubo_robot
{
public:
	Aubo_robot();
	~Aubo_robot();

	Eigen::Matrix3d rotationMatrix_T2B(std::vector<double> theta_z);
	Eigen::Matrix3d rotationMatrix_T2B(double *theta_z);

	Eigen::Matrix<double, 4, 4> T_para(const double &theta, const double &d, const double &a, const double &alpha);

	std::vector<double> robot_fk(std::vector<double> theta_z);

	Eigen::Matrix<double, 6, 6> Jacobi(std::vector<double> theta_z);

	std::vector<double> robot_TcpSpeed(std::vector<double> theta_z, std::vector<double> qd);

	// void motionControlThread(const std::vector<double> &pose);

private:
	Eigen::Matrix<double, 6, 1> robot_a;
	Eigen::Matrix<double, 6, 1> robot_d;
	Eigen::Matrix<double, 6, 1> robot_alpha;
	Eigen::Matrix<double, 6, 1> robot_offset;

	double control_period_ = 5;
	bool enable_motion_control_ = 1;
};

//-------------------------------------------------------

Eigen::MatrixXd pinv(Eigen::MatrixXd A);

Eigen::Matrix<double, 6, 1> Pre_Jcobi(Eigen::Matrix<double, 4, 4> T, Eigen::Matrix<double, 3, 1> p_end);
