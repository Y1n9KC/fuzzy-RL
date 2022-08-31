#include "robot.h"

#define Pi 3.1415926535

Eigen::Matrix<double, 6, 1> Pre_Jcobi(Eigen::Matrix<double, 4, 4> T, Eigen::Matrix<double, 3, 1> p_end)
{
	Eigen::Matrix<double, 6, 1> jacobi;
	Eigen::Matrix<double, 3, 1> jacobi_up;
	Eigen::Vector3d b(T(0, 2), T(1, 2), T(2, 2));
	Eigen::Vector3d p(T(0, 3), T(1, 3), T(2, 3));
	Eigen::Vector3d p_end_temp(p_end(0), p_end(1), p_end(2));
	jacobi_up = b.cross(p_end_temp - p);
	jacobi << jacobi_up(0), jacobi_up(1), jacobi_up(2), b(0), b(1), b(2);
	return jacobi;
}

Aubo_robot::Aubo_robot()
{
	this->robot_d << 0.0985, 0, 0, 0.1215, 0.1025, 0.094;
	this->robot_a << 0, -0.408, -0.376, 0, 0, 0;
	this->robot_alpha << Pi / 2, Pi, Pi, -Pi / 2, Pi / 2, 0;
	this->robot_offset << 0, -Pi / 2, 0, Pi / 2, 0, 0;
}

Aubo_robot::~Aubo_robot()
{
}

Eigen::Matrix<double, 4, 4> Aubo_robot::T_para(const double &theta, const double &d, const double &a, const double &alpha)
{
	Eigen::Matrix<double, 4, 4> T_Matrix;
	T_Matrix.row(0) << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
	T_Matrix.row(1) << sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
	T_Matrix.row(2) << 0, sin(alpha), cos(alpha), d;
	T_Matrix.row(3) << 0, 0, 0, 1;
	return T_Matrix;
}

Eigen::Matrix3d Aubo_robot::rotationMatrix_T2B(std::vector<double> theta_z)
{
	Eigen::Matrix<double, 4, 4> T_01;
	Eigen::Matrix<double, 4, 4> T_12;
	Eigen::Matrix<double, 4, 4> T_23;
	Eigen::Matrix<double, 4, 4> T_34;
	Eigen::Matrix<double, 4, 4> T_45;
	Eigen::Matrix<double, 4, 4> T_56;
	Eigen::Matrix<double, 4, 4> T;
	Eigen::Matrix3d rotation_matrix1;
	std::vector<double> TCP_pose(6, 0);

	for (int i = 0; i < 6; i++)
	{
		theta_z[i] = theta_z[i] + this->robot_offset[i];
	}

	T_01 = this->T_para(theta_z[0], this->robot_d[0], this->robot_a[0], this->robot_alpha[0]);
	T_12 = this->T_para(theta_z[1], this->robot_d[1], this->robot_a[1], this->robot_alpha[1]);
	T_23 = this->T_para(theta_z[2], this->robot_d[2], this->robot_a[2], this->robot_alpha[2]);
	T_34 = this->T_para(theta_z[3], this->robot_d[3], this->robot_a[3], this->robot_alpha[3]);
	T_45 = this->T_para(theta_z[4], this->robot_d[4], this->robot_a[4], this->robot_alpha[4]);
	T_56 = this->T_para(theta_z[5], this->robot_d[5], this->robot_a[5], this->robot_alpha[5]);
	T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

	rotation_matrix1 << T(0, 0), T(0, 1), T(0, 2),
		T(1, 0), T(1, 1), T(1, 2),
		T(2, 0), T(2, 1), T(2, 2);
	return rotation_matrix1;
}

Eigen::Matrix3d Aubo_robot::rotationMatrix_T2B(double *theta_z)
{
	Eigen::Matrix<double, 4, 4> T_01;
	Eigen::Matrix<double, 4, 4> T_12;
	Eigen::Matrix<double, 4, 4> T_23;
	Eigen::Matrix<double, 4, 4> T_34;
	Eigen::Matrix<double, 4, 4> T_45;
	Eigen::Matrix<double, 4, 4> T_56;
	Eigen::Matrix<double, 4, 4> T;
	Eigen::Matrix3d rotation_matrix1;
	std::vector<double> TCP_pose(6, 0);

	for (int i = 0; i < 6; i++)
	{
		theta_z[i] = theta_z[i] + this->robot_offset[i];
	}

	T_01 = this->T_para(theta_z[0], this->robot_d[0], this->robot_a[0], this->robot_alpha[0]);
	T_12 = this->T_para(theta_z[1], this->robot_d[1], this->robot_a[1], this->robot_alpha[1]);
	T_23 = this->T_para(theta_z[2], this->robot_d[2], this->robot_a[2], this->robot_alpha[2]);
	T_34 = this->T_para(theta_z[3], this->robot_d[3], this->robot_a[3], this->robot_alpha[3]);
	T_45 = this->T_para(theta_z[4], this->robot_d[4], this->robot_a[4], this->robot_alpha[4]);
	T_56 = this->T_para(theta_z[5], this->robot_d[5], this->robot_a[5], this->robot_alpha[5]);
	T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

	rotation_matrix1 << T(0, 0), T(0, 1), T(0, 2),
		T(1, 0), T(1, 1), T(1, 2),
		T(2, 0), T(2, 1), T(2, 2);
	return rotation_matrix1;
}
std::vector<double> Aubo_robot::robot_fk(std::vector<double> theta_z)
{
	Eigen::Matrix<double, 4, 4> T_01;
	Eigen::Matrix<double, 4, 4> T_12;
	Eigen::Matrix<double, 4, 4> T_23;
	Eigen::Matrix<double, 4, 4> T_34;
	Eigen::Matrix<double, 4, 4> T_45;
	Eigen::Matrix<double, 4, 4> T_56;
	Eigen::Matrix<double, 4, 4> T;
	Eigen::Matrix3d rotation_matrix1;
	std::vector<double> TCP_pose(6, 0);

	for (int i = 0; i < 6; i++)
	{
		theta_z[i] = theta_z[i] + this->robot_offset[i];
	}

	T_01 = this->T_para(theta_z[0], this->robot_d[0], this->robot_a[0], this->robot_alpha[0]);
	T_12 = this->T_para(theta_z[1], this->robot_d[1], this->robot_a[1], this->robot_alpha[1]);
	T_23 = this->T_para(theta_z[2], this->robot_d[2], this->robot_a[2], this->robot_alpha[2]);
	T_34 = this->T_para(theta_z[3], this->robot_d[3], this->robot_a[3], this->robot_alpha[3]);
	T_45 = this->T_para(theta_z[4], this->robot_d[4], this->robot_a[4], this->robot_alpha[4]);
	T_56 = this->T_para(theta_z[5], this->robot_d[5], this->robot_a[5], this->robot_alpha[5]);
	T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

	rotation_matrix1 << T(0, 0), T(0, 1), T(0, 2),
		T(1, 0), T(1, 1), T(1, 2),
		T(2, 0), T(2, 1), T(2, 2);

	TCP_pose[0] = T(0, 3);
	TCP_pose[1] = T(1, 3);
	TCP_pose[2] = T(2, 3);

	Eigen::Vector3d eulerAngle1 = rotation_matrix1.eulerAngles(2, 1, 0);
	TCP_pose[5] = eulerAngle1[0];
	TCP_pose[4] = eulerAngle1[1];
	TCP_pose[3] = eulerAngle1[2];
	return TCP_pose;
}

Eigen::Matrix<double, 6, 6> Aubo_robot::Jacobi(std::vector<double> theta_z)
{

	for (int i = 0; i < 6; i++)
	{
		theta_z[i] = theta_z[i] + this->robot_offset[i];
	}
	//-------------end-----------

	Eigen::Matrix<double, 4, 4> T_01;
	Eigen::Matrix<double, 4, 4> T_12;
	Eigen::Matrix<double, 4, 4> T_23;
	Eigen::Matrix<double, 4, 4> T_34;
	Eigen::Matrix<double, 4, 4> T_45;
	Eigen::Matrix<double, 4, 4> T_56;
	T_01 = this->T_para(theta_z[0], this->robot_d[0], this->robot_a[0], this->robot_alpha[0]);
	T_12 = this->T_para(theta_z[1], this->robot_d[1], this->robot_a[1], this->robot_alpha[1]);
	T_23 = this->T_para(theta_z[2], this->robot_d[2], this->robot_a[2], this->robot_alpha[2]);
	T_34 = this->T_para(theta_z[3], this->robot_d[3], this->robot_a[3], this->robot_alpha[3]);
	T_45 = this->T_para(theta_z[4], this->robot_d[4], this->robot_a[4], this->robot_alpha[4]);
	T_56 = this->T_para(theta_z[5], this->robot_d[5], this->robot_a[5], this->robot_alpha[5]);
	Eigen::Matrix<double, 4, 4> T_0;
	Eigen::Matrix<double, 4, 4> T_1;
	Eigen::Matrix<double, 4, 4> T_2;
	Eigen::Matrix<double, 4, 4> T_3;
	Eigen::Matrix<double, 4, 4> T_4;
	Eigen::Matrix<double, 4, 4> T_5;
	Eigen::Matrix<double, 4, 4> T_6;
	T_1 = T_01;
	T_2 = T_1 * T_12;
	T_3 = T_2 * T_23;
	T_4 = T_3 * T_34;
	T_5 = T_4 * T_45;
	T_6 = T_5 * T_56;
	T_0.row(0) << 0, 0, 0, 0;
	T_0.row(1) << 0, 0, 0, 0;
	T_0.row(2) << 0, 0, 1, 0;
	T_0.row(3) << 0, 0, 0, 0;

	Eigen::Matrix<double, 6, 6> jacobi;
	Eigen::Matrix<double, 6, 1> jacobi_1;
	Eigen::Matrix<double, 6, 1> jacobi_2;
	Eigen::Matrix<double, 6, 1> jacobi_3;
	Eigen::Matrix<double, 6, 1> jacobi_4;
	Eigen::Matrix<double, 6, 1> jacobi_5;
	Eigen::Matrix<double, 6, 1> jacobi_6;

	Eigen::Matrix<double, 3, 1> b;
	b << 0, 0, 1;
	Eigen::Matrix<double, 3, 1> b_n;
	b_n << T_6(0, 3), T_6(1, 3), T_6(2, 3);

	jacobi_1 = Pre_Jcobi(T_0, b_n);
	jacobi_2 = Pre_Jcobi(T_1, b_n);
	jacobi_3 = Pre_Jcobi(T_2, b_n);
	jacobi_4 = Pre_Jcobi(T_3, b_n);
	jacobi_5 = Pre_Jcobi(T_4, b_n);
	jacobi_6 = Pre_Jcobi(T_5, b_n);
	for (int i = 0; i < 6; i++)
	{
		jacobi(i, 0) = jacobi_1(i);
		jacobi(i, 1) = jacobi_2(i);
		jacobi(i, 2) = jacobi_3(i);
		jacobi(i, 3) = jacobi_4(i);
		jacobi(i, 4) = jacobi_5(i);
		jacobi(i, 5) = jacobi_6(i);
	}
	return jacobi;
}

std::vector<double> Aubo_robot::robot_TcpSpeed(std::vector<double> theta_z, std::vector<double> qd)
{
	std::vector<double> v(6, 0);
	Eigen::Matrix<double, 6, 1> v_temp;
	Eigen::Matrix<double, 6, 1> qd_temp;
	Eigen::Matrix<double, 6, 6> jacobi;
	qd_temp << qd[0], qd[1], qd[2], qd[3], qd[4], qd[5];
	jacobi = this->Jacobi(theta_z);
	v_temp = jacobi * qd_temp;
	for (int i = 0; i < 6; i++)
	{
		v[i] = v_temp(i);
	}
	return v;
}

Eigen::MatrixXd pinv(Eigen::MatrixXd A)
{

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // M=USV*
	double pinvtoler = 1.e-8;															 // tolerance
	int row = A.rows();
	int col = A.cols();
	int k = min(row, col);
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
	Eigen::MatrixXd singularValues_inv = svd.singularValues();
	Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	for (long i = 0; i < k; ++i)
	{
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else
			singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose()); // X=VS+U*
	return X;
}
