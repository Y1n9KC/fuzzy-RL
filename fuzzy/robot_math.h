#pragma once
#include <iostream>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#define Pi 3.1415926535

using namespace std;
using namespace Eigen;

//-----------------滤波器类--------------
class Lp1Filter
{
public:
	Lp1Filter(float f_cut, float dT);
	~Lp1Filter();
	//使用低通滤波
	double run(const double &input);

private:
	float RC;	  //模电RC滤波的电阻与电容的乘积
	float dT;	  //控制周期（s）
	float k;	  //滤波系数
	double state; //滤波结果
};

class SWA_Filter

{
private:
	int window_size;
	list<double> record;
	int num = 0;

public:
	SWA_Filter(int window_size);
	~SWA_Filter();
	double run(const double &input);
};

//-----------------------PID类------------------
class PID
{
public:
	PID();
	~PID();
	// pid初始化
	void PID_init(const double &Kp, const double &Ki, const double &Kd, const double &cal_time);
	// pid运行
	double PID_run(const double &pid_desired, const double &input);
	Eigen::Matrix<float, 6, 1> PID_run(Eigen::Matrix<float, 6, 1> pid_desired, Eigen::Matrix<float, 6, 1> input);

	double PID_increment_run(const double &pid_desired, const double &input);
	// pidshow
	void PID_show();
	void PID_integral_setzero();

private:
	double Kp;
	double Ki;
	double Kd;
	double cal_time;	//计算周期
	double pid_desired; //期望
	double err;			//偏差
	double last_err;	//上时刻偏差
	double pid_actual;	//实际
	double output;		//输出
	double integral;	//积分
	//矩阵变量定义
	Eigen::Matrix<float, 6, 1> Matrix_pid_desired;
	Eigen::Matrix<float, 6, 1> Matrix_err;
	Eigen::Matrix<float, 6, 1> Matrix_pid_actual;
	Eigen::Matrix<float, 6, 1> Matrix_output;
	Eigen::Matrix<float, 6, 1> Matrix_integral;
	Eigen::Matrix<float, 6, 1> Matrix_last_err;
	//增加低通滤波器类
};

//-------------------------五次多项式规划----------------------
class Quintic_Polynomial_Plan
{
public:
	Quintic_Polynomial_Plan(double time_start,
							double time_end,
							double position_start,
							double position_end,
							double velocity_start,
							double velocity_end,
							double accelerate_start,
							double accelerate_end,
							int part);

	Quintic_Polynomial_Plan(double time_end, double position_end, double dt);
	~Quintic_Polynomial_Plan();
	double get_time_end() { return time_end; }
	void coefficient();
	double move(int times);
	std::vector<double> getK();

private:
	double time_start;
	double time_end;
	double position_start;
	double position_end;
	double velocity_start;
	double velocity_end;
	double accelerate_start;
	double accelerate_end;
	int part; //分割的份数
	double k0, k1, k2, k3, k4, k5;
};
