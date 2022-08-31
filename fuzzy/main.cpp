#include <aubo_driver/aubo_driver.h>

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream> //读写文件
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <queue>
#include <time.h>
#include <thread>
#include <mutex>
#include <future>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "robot.h"
#include "robot_math.h"
#include "Fuzzy.h"
#include "fuzzy_learning.h"
// #include "StbObserver.h"
//#include "optoclient.h"
//确定使用内部或者外部力传感器，内部定义0，外部定义1(ati)
#define external_sensor 1
// #define stable_observer 1
#define FIS 1
#define FQL 0

// AuboServer地址为192.168.1.104
#define SERVER_IPADDR "192.168.1.100"
#define SERVER_PORT 30001
#define ROBOT_ID "user"
#define ROBOT_PASSWORD "111"

// ati力传感器地址为192.168.1.23
#define SENSOR_ADDRESS "192.168.1.23"
#define SENSOR_PORT 49152
#define SENSOR_COMMAND 2 // streaming
#define NUM_SAMPLE 1

//数学参数
#define Pi 3.14159265358979323846
mutex m_lock; //实例化线程锁对象 m_lock
using namespace arcs::aubo_driver;
using namespace std;
using namespace std::chrono;

// FILE *recordFile = fopen("record.txt", "w");

double F_sensor_time = 1.0 / 1000.0 * 1000000; //定义时间,单位ms
double cal_times = 200.0;					   //定义计算频率
double cal_dt = 1.0 / cal_times;			   //定义计算时间常数
double cal_time = cal_dt * 1000000;			   //定义计算循环时间,单位ms
int send_times = cal_times / (200.0);		   //定义发送频率
double fliter_alpha = 0.1;

#if external_sensor
// ati力传感器数据类型
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct response_struct
{
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;
#endif
//返回距离
double distance(const std::vector<double> &a, const std::vector<double> &b)
{
	double res = 0.;
	if (a.size() != b.size())
	{
		return -1;
	}
	for (int i = 0; i < (int)a.size(); i++)
	{
		res += (a[i] - b[i]) * (a[i] - b[i]);
	}
	return sqrt(res);
}
//运动函数
void setservomove(AuboDriver *m_instance, const std::vector<double> &pose)
{
	m_lock.lock();
	// 发送轨迹点到机器人
	auto builder = m_instance->getRtdeInputBuilder();
	builder->servoCartesian(pose);
	builder->send();

	m_lock.unlock();
	//执行完关闭
}
//计算时间
void timespec_add_us(struct timespec *t, long us)
{
	t->tv_nsec += us * 1000;
	if (t->tv_nsec > 1000000000)
	{
		t->tv_nsec = t->tv_nsec - 1000000000; // + ms*1000000;
		t->tv_sec += 1;
	}
}

/*-----------------------------------------------------------------------------------------------------------------*/

int main()
{

	//自定义机器人实例化(用于正逆运动学)
	Aubo_robot robot_aubo;

	//实例化五次多项式类

	Quintic_Polynomial_Plan Move1(1, -0.03, 200 * 5);
	Quintic_Polynomial_Plan Move2(1, -0.05, 160);
	//实例化滤波器类
	Lp1Filter Filter_jerk(50, 0.005); // 50hz
	SWA_Filter SWA_jerk(50);
	//实例化pid类
	PID Pid1;
	PID Pid2;
	Pid1.PID_init(0.9, 0.0, 0.00, 0.005); //迪卡尔位置环参数初始化
	Pid2.PID_init(1, 0, 0, 0.005);		  //力外环pid参数初始化
	cout << "相关类实例化成功。" << endl;
	//寄存器订阅变量
	vector<string> name;
	//定义获取数据所需变量
	std::vector<double> actual_TCP_pose;	 //实际tcp位置
	std::vector<double> actual_TCP_speed;	 //实际tcp速度
	std::vector<double> actual_Joint_speed;	 //实际关节速度
	std::vector<double> record_x;			 // x方向位置记录
	std::vector<double> record_y;			 // y方向位置记录
	std::vector<double> record_z;			 // z方向位置记录
	std::vector<double> record_xout(6, 0.0); //记录计算点位置
	std::vector<double> record_speed_x;
	std::vector<double> record_speed_y;
	std::vector<double> record_speed_z;
	std::vector<double> record_accelerate_x;
	std::vector<double> record_accelerate_y;
	std::vector<double> record_accelerate_z;
	std::vector<double> record_jerk_x;
	std::vector<double> record_jerk_y;
	std::vector<double> record_jerk_z;
	std::vector<double> record_fujia;
	double Kp_alpha = 0.61; // Kp,截至频率50hz
	double alpha = 0.1;		//滤波系数
	double F_alpha = 0.1;	// F滤波系数
	double fujia_speed = 0;
	double speed_x = 0;
	double speed_y = 0;
	double speed_z = 0;
	double accelerate_x = 0;
	double accelerate_y = 0;
	double accelerate_z = 0;
	double jerk_x = 0;
	double jerk_y = 0;
	double jerk_z = 0;
	int position_index = 0;
	int speed_index = 0;
	int accelerate_index = 0;
	int jerk_index = 0;
	double current_acc = 0;
	double last_acc = 0;
	float D_x = 70;
	float D_y = 40;
	double fujia_distance = 0;
	//定义多线程所需的变量
	std::promise<int> promise_end;							//线程完成标志值实例化
	std::future<int> future_end = promise_end.get_future(); //将future与promise链接

	//机械臂导纳属性
	float KX = 0;
	float KY = 0;
	float KZ = 0;
	float DX = 100;
	float DY = 10000000;
	float DZ = 10000000;
	float MX = 20;
	float MY = 20000000;
	float MZ = 2000000;
	float RK = 10000000;
	float RD = 10000000;
	float RM = 50000000;

	//模糊相关变量
	vector<double> state;
	int start_looptime = 0, end_looptime = 0;

	// char filebuffer[128];
	// int file_num;
	// char file_tail[5] = ".txt";
	//改变记录文件名
	// fstream numrecord;
	// numrecord.open("numrecord.txt", ios::in | ios::out);

	// numrecord.getline(filebuffer, 50);
	// sscanf(filebuffer, "%d ", file_num);

	// numrecord.close();

	// char file_name = to_string(file_num) + ".txt";
	// memset(file_name, 0, sizeof(file_name));
	// itoa(file_num, file_name, 10);
	// strcat(file_name, file_tail);

	FILE *recordFile = fopen("record.txt", "w");

#if FIS
	double Force_area[7] = {-45, -30, -15, 0, 15, 30, 45};
	double speed_area[7] = {-0.45, -0.30, -0.15, 0, 0.15, 0.30, 0.45};
	int RULE1[7][7] = {{0, 1, 2, 3, 4, 5, 6},
					   {1, 2, 3, 4, 5, 6, 5},
					   {2, 3, 4, 5, 6, 5, 4},
					   {3, 4, 5, 6, 5, 4, 3},
					   {4, 5, 6, 5, 4, 3, 2},
					   {5, 6, 5, 4, 3, 2, 1},
					   {6, 5, 4, 3, 2, 1, 0}};
	double Damping_Area1[7] = {50, 75, 100, 125, 150, 200, 300};

	double intention_area[9] = {-20, -15, -10, -5, 0, 5, 10, 15, 20};
	double cos_area[5] = {0, 0.1, 0.2, 0.5, 1};
	int RULE2[5][9] = {{8, 8, 7, 7, 6, 7, 7, 8, 8},
					   {7, 7, 6, 6, 5, 5, 5, 4, 3},
					   {7, 7, 6, 6, 5, 5, 4, 3, 2},
					   {7, 7, 6, 6, 5, 4, 3, 2, 1},
					   {6, 6, 5, 5, 4, 3, 2, 1, 0}};
	double Damping_Area2[9] = {50, 75, 100, 125, 150, 175, 200, 250, 300};
	double intention_x, intention_y;
	double squ_plus;
	double cos_x, cos_y;

	double adaptive_damping_x, adaptive_damping_y;
#endif
	double total_reward = 0.0;
	double temp_reward = 0.0;
#if FQL
	//模糊参数设定
	const vector<double> Force_Area = {-40, -20, 0, 20, 40};
	const vector<double> velocity_Area = {-0.50, -0.25, 0, 0.25, 0.50};
	const vector<double> acc_Area = {-1.2, -0.6, 0, 0.6, 1.2};
	// const vector<double> dotF_Area = {-100, -50, -25, 0, 25, 50, 100};
	const vector<double> cosine_Area = {0, 0.1, 0.2, 0.5, 1};
	const vector<double> damping_Area = {35, 50, 75, 100};
	// const vector<double> damping_Area = {100, 100, 100, 100};
	const vector<vector<double>> input_area = {Force_Area, velocity_Area, acc_Area};

	//状态数计算
	int state_num = 1;
	for (int i = 0; i < input_area.size(); i++)
	{
		state_num *= input_area[i].size();
	}
	//实例化模糊RL类
	Reinforcement_Learning fuzzy_RL(input_area, damping_Area, 0.1, 0.9, 0.95, 0.10); //后四个参数beta,lambda,gamma,greedy
	bool stop_flag = true;
	bool learning_flag = false;
	bool back_flag = false;
	int record_flag = 0;
	double reward = 0.0;
	// int start_looptime = 0, end_looptime = 0;

	double last_damping;
	vector<double> phi;

	double Q_val, Q_star;
	// vector<vector<double>> Q_temp(state_num, vector<double>(damping_Area.size()));
	// vector<vector<double>> E_temp(state_num, vector<double>(damping_Area.size()));
	fstream Q_table;	 // Q表记录
	fstream eligibility; //资格迹记录
	srand(time(NULL));
	char buffer[256];

#endif
	//------------------------------------------------------------------
	//设置要实现的惯量矩阵
	Eigen::Matrix<float, 6, 6> M_matrix_66;
	M_matrix_66.row(0) << MX, 0, 0, 0, 0, 0;
	M_matrix_66.row(1) << 0, MY, 0, 0, 0, 0;
	M_matrix_66.row(2) << 0, 0, MZ, 0, 0, 0;
	M_matrix_66.row(3) << 0, 0, 0, RM, 0, 0;
	M_matrix_66.row(4) << 0, 0, 0, 0, RM, 0;
	M_matrix_66.row(5) << 0, 0, 0, 0, 0, RM;
	//阻尼矩阵
	Eigen::Matrix<float, 6, 6> B_matrix_66;
	B_matrix_66.row(0) << DX, 0, 0, 0, 0, 0;
	B_matrix_66.row(1) << 0, DY, 0, 0, 0, 0;
	B_matrix_66.row(2) << 0, 0, DZ, 0, 0, 0;
	B_matrix_66.row(3) << 0, 0, 0, RD, 0, 0;
	B_matrix_66.row(4) << 0, 0, 0, 0, RD, 0;
	B_matrix_66.row(5) << 0, 0, 0, 0, 0, RD;
	//刚度矩阵
	Eigen::Matrix<float, 6, 6> K_matrix_66;
	K_matrix_66.row(0) << KX, 0, 0, 0, 0, 0;
	K_matrix_66.row(1) << 0, KY, 0, 0, 0, 0;
	K_matrix_66.row(2) << 0, 0, KZ, 0, 0, 0;
	K_matrix_66.row(3) << 0, 0, 0, RK, 0, 0;
	K_matrix_66.row(4) << 0, 0, 0, 0, RK, 0;
	K_matrix_66.row(5) << 0, 0, 0, 0, 0, RK;

	//定义当前和前一次的误差值 d dd
	Eigen::Matrix<float, 6, 1> current_pose_error;
	current_pose_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> dot_current_pose_error;
	dot_current_pose_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> ddot_current_pose_error;
	ddot_current_pose_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> last_pose_error;
	last_pose_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> dot_last_pose_error;
	dot_last_pose_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> ddot_last_pose_error;
	ddot_last_pose_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> F_matrix_61;
	F_matrix_61 << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> lastF_matrix_61;
	F_matrix_61 << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> dotF_matrix_61;
	F_matrix_61 << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> Fc_matrix_61;
	Fc_matrix_61 << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> F_matrix_sum; //用于低通滤波
	F_matrix_sum << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> reference_pose;
	reference_pose << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> F_d;
	F_d << 0, 0, 0, 0, 0, 0;
	//定义记录变量与返回变量
	Eigen::Matrix<float, 6, 1> record_TCP_pose;
	record_TCP_pose << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> cal_dot_TCP;
	cal_dot_TCP << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> cal_ddot_TCP;
	cal_ddot_TCP << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> F_matrix_back;
	F_matrix_back << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> F_back;
	F_back << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> F_error;
	F_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> back_pose_error;
	back_pose_error << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> back_dot_current_pose;
	back_dot_current_pose << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<float, 6, 1> back_ddot_current_pose;
	back_ddot_current_pose << 0, 0, 0, 0, 0, 0;
	//旋转变换所需变量
	Eigen::Matrix3d rotation_matrix_dR; //最后所得旋转矩阵
	Eigen::Matrix3d rotation_matrix_Rr;
	Eigen::Matrix3d rotation_matrix_R0; //平衡位置旋转矩阵
	Eigen::Matrix4d rotation_matrix_T;
	std::vector<double> xout(6, 0.);
	std::vector<double> qnear(6, 0.); //逆解需要的初始参数
	std::vector<double> pose_temp;	  //临时发送位置变量
	std::vector<double> qout_joint;
	std::vector<double> TargetAccelerations;

	//定义环境辨识所需的参数
	// double Xe_0 = -0.00;
	// double Ke_0 = 2500;
	// double r1 = 300000;
	// double r2 = 1;
	// double Xe = 0, Ke = 0;
	// double hat_f = 0;
	// double hat_Xe = Xe_0;
	// double hat_Ke = Ke_0;
	// double Ke_temp = 0;
	// double Xe_temp = 0;
	// double Xr_distance = 0;
	Eigen::Matrix<float, 6, 1> X_r;
	X_r << 0, 0, 0, 0, 0, 0;
	// double ddot_X_rd = 0;
	// double dot_X_rd = 0;
	// double X_rd = 0;
	// double last_X_rd = 0;
	// double dot_last_X_rd = 0;
	// double last_reference = 0;
#if external_sensor
	//定义ATI力传感器所需的变量
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	int socketHandle;
	struct sockaddr_in addr;
	struct hostent *he;
	byte request[8];
	RESPONSE resp;
	byte response[36];
	int index;
	int sensor_err;
	char *AXES[] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1)
	{
		printf("力传感器连接失败.\n");
		exit(1);
	}

	*(uint16 *)&request[0] = htons(0x1234);
	*(uint16 *)&request[2] = htons(SENSOR_COMMAND);
	*(uint32 *)&request[4] = htonl(NUM_SAMPLE);

	he = gethostbyname(SENSOR_ADDRESS);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(SENSOR_PORT);

	printf("connect.\n");
	sensor_err = connect(socketHandle, (struct sockaddr *)&addr, sizeof(addr));
	if (sensor_err == -1)
	{
		printf("sensor_err: %d.\n", sensor_err);
		exit(2);
	}
#endif

	//设置机器人的速度与加速度
	double j_acc = 1;
	double j_velc = 1;
	double l_acc = 0.1;
	double l_velc = 0.1;
	double l_r = 1;
	vector<double> current_joint;
	//实例化aubo_driver
	auto aubo_driver = createAuboDriver();

	//--------------------------------变量定义结束---------------------------------------

	//----------------**获取 rtde 的输入输出配方**--------------------
	map<string, string> out_list = aubo_driver->getRtdeOutputList();
	map<string, string> in_list = aubo_driver->getRtdeInputList();
	for (auto it : out_list)
	{
		cout << "输出配方：" << it.first << ":" << it.second << endl;
	}

	for (auto it : in_list)
	{
		cout << "输入配方：" << it.first << ":" << it.second << endl;
	}

	aubo_driver->connectToServer("192.168.1.104", 30001, 200);

	if (aubo_driver->login("user", "111", 50))
	{
		cout << "login success" << endl;

		aubo_driver->setRobotPowerOn();
		aubo_driver->setRobotOperational();
		name.push_back("actual_q");
		name.push_back("actual_qd");
		name.push_back("actual_TCP_speed");
		name.push_back("actual_TCP_pose");
		name.push_back("actual_TCP_force");
		aubo_driver->setRtdeOutputRecipe(0, name, 200);
		aubo_driver->startRtdeTransmission(0);
		aubo_driver->syncRtdeTransmission(0, 500);
		try
		{
			current_joint = aubo_driver->getJointPositions();
		}
		catch (...)
		{
		}
		name.clear();
		name.push_back("actual_TCP_speed");
		name.push_back("actual_TCP_pose");
		aubo_driver->setRtdeOutputRecipe(2, name, 200);
		aubo_driver->startRtdeTransmission(2);
		aubo_driver->syncRtdeTransmission(2, 500);
	}
	//------------------在平稳时刻记录力传感器初值---------------
	// Eigen::Matrix<double, 6, 1> ATI_F;
	// ATI_F << 0, 0, 0, 0, 0, 0;
	// Eigen::Matrix<double, 6, 1> F_zero;
	// F_zero << 0, 0, 0, 0, 0, 0;

	cout << "teach point" << endl;
	vector<double> teach_point;

	teach_point.push_back(-0.0 * Pi / 180);
	teach_point.push_back(33.99 * Pi / 180);
	teach_point.push_back(101.86 * Pi / 180);
	teach_point.push_back(-21.01 * Pi / 180);
	teach_point.push_back(90.89 * Pi / 180);
	teach_point.push_back(-54.88 * Pi / 180);

	vector<double> tcp_teach_point = {407.53, -282.04, 342.34, 0.00, 0.00, 0.00};

	aubo_driver->moveJoint(teach_point, j_acc, j_velc);
	while (true)
	{
		try
		{
			current_joint = aubo_driver->getJointPositions();
			if ((current_joint.size() == aubo_driver->getRobotDof()))
			{
				double dis = distance(current_joint, teach_point);
				printf("--  %.6lf  --\n", dis);
				if (dis < 0.001)
				{
					break;
				}
			}
		}
		catch (ValueExpired e)
		{
			cout << "getRobotDof: " << aubo_driver->getRobotDof();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	cout << "reach teach point" << endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// auto mode = aubo_driver->getRobotControlMode();
	// cout << mode << endl;

	//---------------------------获得平衡位置------------------------
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	current_joint = aubo_driver->getJointPositions();
	std::vector<double> current_pose;
	for (int i = 0; i < 6; i++)
	{
		current_pose.push_back(current_joint[i]);
	}
	//测试
	printf("平衡位置角度:  ");
	for (int i = 0; i < 6; i++)
	{
		printf("%.6lf   ", current_pose[i]);
	}
	printf("\n");
	//正解得到末端位置，与旋转矩阵
	auto equivalent_pose = aubo_driver->getForwardKinematics(current_pose);
	printf("平衡位置笛卡尔坐标:  ");
	for (int i = 0; i < 6; i++)
	{
		printf("%.6lf   ", equivalent_pose[i]);
	}
	printf("\n");
	Eigen::AngleAxisd rollAngle_R0(Eigen::AngleAxisd(equivalent_pose[5], Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle_R0(Eigen::AngleAxisd(equivalent_pose[4], Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle_R0(Eigen::AngleAxisd(equivalent_pose[3], Eigen::Vector3d::UnitZ()));
	rotation_matrix_R0 = yawAngle_R0 * pitchAngle_R0 * rollAngle_R0;
	current_pose.clear(); //每次使用完清空
	printf("控制循环开始.\n");
#if external_sensor
	Eigen::Matrix<double, 6, 1> ATI_F;
	ATI_F << 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<double, 6, 1> F_zero;
	F_zero << 0, 0, 0, 0, 0, 0;
	//获得力的初值
	send(socketHandle, request, 8, 0);
	recv(socketHandle, response, 36, 0);
	resp.rdt_sequence = ntohl(*(uint32 *)&response[0]);
	resp.ft_sequence = ntohl(*(uint32 *)&response[4]);
	resp.status = ntohl(*(uint32 *)&response[8]);
	for (int i = 0; i < 6; i++)
	{
		resp.FTData[i] = ntohl(*(int32 *)&response[12 + i * 4]);
	}
	for (int i = 0; i < 6; i++)
	{
		F_zero[i] = resp.FTData[i] / 1000000.0;
	}

#endif
	int loop_times = 1; //循环次数
	// int temp_times = 1;
	int F_times = 1;
	auto start = high_resolution_clock::now(); //开始时间戳
	//记录当前绝对时间
	struct timespec next;
	clock_gettime(CLOCK_REALTIME, &next);
	//固定时间间隔
	struct timespec duration;
	duration.tv_nsec = 5 * 1000 * 1000;

	// vector<double> F_internal;
	// //获取传感器数值
	// F_internal = aubo_driver->getTcpForce();
	// for (int i = 0; i < 6; i++)
	// {
	// 	ATI_F[i] = F_internal[i];
	// }
	// for (int i = 0; i < 6; i++)
	// {
	// 	F_zero[i] = F_internal[i];
	// }

	/*----------------------------控制循环开始-------------------------------------*/
	while (true)
	{
		//循环周期控制
		F_times++;
//力传感器读取
#if external_sensor
		send(socketHandle, request, 8, 0);
		recv(socketHandle, response, 36, 0);
		resp.rdt_sequence = ntohl(*(uint32 *)&response[0]);
		resp.ft_sequence = ntohl(*(uint32 *)&response[4]);
		resp.status = ntohl(*(uint32 *)&response[8]);
		for (int i = 0; i < 6; i++)
		{
			resp.FTData[i] = ntohl(*(int32 *)&response[12 + i * 4]);
		}
		//获取末端力并减去初始误差
		for (int i = 0; i < 6; i++)
		{
			ATI_F[i] = resp.FTData[i] / 1000000.0 - F_zero[i];
		}
		//处理力的方向
		F_matrix_61[0] = ATI_F[0];
		F_matrix_61[1] = -ATI_F[1];
		F_matrix_61[2] = -ATI_F[2];
		F_matrix_61[3] = ATI_F[3];
		F_matrix_61[4] = -ATI_F[4];
		F_matrix_61[5] = -ATI_F[5];
#else
		//获取内部力传感器
		F_internal = aubo_driver->getTcpForce();
		for (int i = 0; i < 6; i++)
		{
			ATI_F[i] = F_internal[i];
			ATI_F[i] -= F_zero[i]; //减去初值
		}
		//处理力的方向
		F_matrix_61[0] = ATI_F[1];
		F_matrix_61[1] = ATI_F[0];
		F_matrix_61[2] = -ATI_F[2];
		F_matrix_61[3] = ATI_F[3];
		F_matrix_61[4] = ATI_F[4];
		F_matrix_61[5] = ATI_F[5];
#endif
		//一阶低通滤波器
		for (int i = 0; i < 6; i++)
		{
			F_matrix_sum[i] = F_alpha * F_matrix_61[i] + (1 - F_alpha) * F_matrix_sum[i];
		}

		//
		for (int i = 0; i < 6; i++)
		{
			F_matrix_61[i] = F_matrix_sum[i];
		}
		//如果检测力太小则设为0
		// cout << "力数据：" << endl;
		// for (int i = 0; i < 6; i++)
		// {
		// 	if (abs(F_matrix_61[i]) < 0.1)
		// 	{
		// 		F_matrix_61[i] = 0;
		// 	}
		// 	cout << F_matrix_61[i] << " ";
		// }
		// cout << endl;
		//力外环pid
		F_error = Pid2.PID_run(F_matrix_61, F_d);

		//----------------------addmittance算法开始---------------------
		//误差数据更新
		dot_last_pose_error = dot_current_pose_error;
		last_pose_error = current_pose_error;
		//算法实现
		ddot_current_pose_error = M_matrix_66.inverse() * ((F_matrix_61)-B_matrix_66 * dot_last_pose_error - K_matrix_66 * (last_pose_error - X_r));
		dot_current_pose_error = dot_last_pose_error + ddot_current_pose_error * cal_dt;
		current_pose_error = last_pose_error + dot_current_pose_error * cal_dt;

		reference_pose[0] = current_pose_error[0];
		reference_pose[1] = current_pose_error[1];
		reference_pose[2] = current_pose_error[2];

		Eigen::Vector3d eulerAngle(current_pose_error[5], current_pose_error[4], current_pose_error[3]);
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
		rotation_matrix_dR = yawAngle * pitchAngle * rollAngle;
		rotation_matrix_Rr = rotation_matrix_dR * rotation_matrix_R0;
		Eigen::Quaterniond quaternion_R0(rotation_matrix_Rr);						  //得到末端姿态的四元数
		Eigen::Vector3d eulerAngle_end = quaternion_R0.matrix().eulerAngles(2, 1, 0); //转为欧拉角

		//------------------增加迪卡尔空间 pd---------------------------------
		//获得当前末端位置
		actual_TCP_pose.clear();
		current_joint.clear();
		current_joint = aubo_driver->getJointPositions();
		actual_TCP_pose = robot_aubo.robot_fk(current_joint); //正运动学

		//---------------得到笛卡尔目标位置--------------
		// 加入平衡位置
		for (int i = 0; i < 6; i++)
		{
			reference_pose[i] += equivalent_pose[i];
		}
		xout[0] = reference_pose[0];
		xout[1] = reference_pose[1];
		xout[2] = reference_pose[2];
		xout[3] = eulerAngle_end[0];
		xout[4] = eulerAngle_end[1];
		xout[5] = eulerAngle_end[2];

		//记录生成的输出队列信息
		for (int i = 0; i < 6; i++)
		{
			record_xout[i] = record_xout[i] + xout[i];
		}

		//--------------------运动控制-----------------------
		// pose_temp.clear();
		// cout << "输出位置: ";
		// for (int i = 0; i < 6; i++)
		// {
		// 	cout << xout[i] << " ";
		// }
		// cout << endl;

		// cout << current_pose_error(0) << endl;

		//笛卡尔空间运动线程
		std::thread set_rtde_thread(setservomove, aubo_driver, xout);
		//与主线程剥离
		if (set_rtde_thread.joinable())
		{
			set_rtde_thread.detach();
		}

		//----------------数据记录----------------
		//获得当前末端位置
		actual_TCP_pose.clear();
		current_joint.clear();
		current_joint = aubo_driver->getJointPositions();
		actual_TCP_pose = robot_aubo.robot_fk(current_joint); //正运动学
		auto record_time = high_resolution_clock::now();
		//-----------------控制循环终止---------------
		timespec_add_us(&next, 5.0 * 1000);							 // unit: user
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL); //按照绝对时间进行休眠
		//获取末端时间戳
		auto end = high_resolution_clock::now();
		auto d_time = duration_cast<chrono::microseconds>(end - start).count();
		auto adcontrol_time = duration_cast<chrono::microseconds>(end - record_time).count();
		// cal_dt=d_time*1000000;//计算时间设置为上个周期的时间

		//---------------对位置微分求速度加速度-------------------
		record_x.push_back(actual_TCP_pose[0]);
		record_y.push_back(actual_TCP_pose[1]);
		record_z.push_back(actual_TCP_pose[2]); //-fujia_distance);
		// cout<<"distance_x:  "<<record_x[loop_times-1]<<"  distance_y:   "
		//<<record_y[loop_times-1]<<"  distance_z:  "<<record_z[loop_times-1]<<endl;

		actual_Joint_speed.clear();
		//获取关节速度
		actual_Joint_speed = aubo_driver->getJointVelocities();
		//转化为tcp速度
		actual_TCP_speed = robot_aubo.robot_TcpSpeed(current_joint, actual_Joint_speed);
		//计算速度
		speed_x = actual_TCP_speed[0];
		speed_y = actual_TCP_speed[1];
		speed_z = actual_TCP_speed[2];

		if (speed_index > 0) //低通滤波,速度滤波从第二个开始
		{
			speed_x = alpha * speed_x + (1 - alpha) * record_speed_x[speed_index - 1];
			speed_y = alpha * speed_y + (1 - alpha) * record_speed_y[speed_index - 1];
			speed_z = alpha * speed_z + (1 - alpha) * record_speed_z[speed_index - 1];
		}
		record_speed_x.push_back(speed_x);
		record_speed_y.push_back(speed_y);
		record_speed_z.push_back(speed_z);

		// TargetAccelerations = aubo_driver->getJointTargetAccelerations();

		//计算加速度
		if (speed_index > 0) //速度有两个开始计算加速度
		{
			accelerate_x = (speed_x - record_speed_x[speed_index - 1]) / cal_dt;
			accelerate_y = (speed_y - record_speed_y[speed_index - 1]) / cal_dt;
			accelerate_z = (speed_z - record_speed_z[speed_index - 1]) * 200;
			if (abs(accelerate_x) > 100) //如果加速度突然变得太大使用上一个数据
			{
				accelerate_x = 0; // record_accelerate_z[accelerate_index];
			}

			if (accelerate_index > 0) //加速度从第二个开始滤波
			{
				accelerate_x = alpha * accelerate_x + (1 - alpha) * record_accelerate_x[accelerate_index - 1];
				accelerate_y = alpha * accelerate_y + (1 - alpha) * record_accelerate_y[accelerate_index - 1];
				accelerate_z = alpha * accelerate_z + (1 - alpha) * record_accelerate_z[accelerate_index - 1];
			}

			record_accelerate_x.push_back(accelerate_x);
			record_accelerate_y.push_back(accelerate_y);
			record_accelerate_z.push_back(accelerate_z);
			accelerate_index++;
		}
		speed_index++;

		// 		double current_acc = 0;
		// double last_acc = 0;
		jerk_x = (current_acc - last_acc) / cal_dt;
		last_acc = current_acc;
		current_acc = accelerate_x;
		// jerk_x = SWA_jerk.run(jerk_x);
		jerk_x = Filter_jerk.run(jerk_x);

		dotF_matrix_61 = (F_matrix_61 - lastF_matrix_61) / cal_dt;
		lastF_matrix_61 = F_matrix_61;

		state = {F_matrix_61[0], actual_TCP_speed[0], dotF_matrix_61[0]};

#if FIS
		Fuzzy fuzzy;

		adaptive_damping_x = fuzzy.fuzzy_damping_Fv(F_matrix_61[0], actual_TCP_speed[0],
													Force_area, speed_area, RULE1, Damping_Area1);
		adaptive_damping_y = fuzzy.fuzzy_damping_Fv(F_matrix_61[1], actual_TCP_speed[1],
													Force_area, speed_area, RULE1, Damping_Area1);
		if (isnan(adaptive_damping_x))
			adaptive_damping_x = 275;
		if (isnan(adaptive_damping_y))
			adaptive_damping_y = 275;

		//力速度乘积与cos角的模糊
		//计算intention和cos
		// intention_x = F_matrix_61[0] * actual_TCP_speed[0];
		// intention_y = F_matrix_61[1] * actual_TCP_speed[1];
		// squ_plus = pow((pow(F_matrix_61[0], 2) + pow(F_matrix_61[1], 2)), 0.5);

		// if (squ_plus == 0)
		// {
		// 	cos_x = 1;
		// 	cos_y = 1;
		// }
		// else
		// {
		// 	cos_x = abs(F_matrix_61[0]) / squ_plus;
		// 	cos_y = abs(F_matrix_61[1]) / squ_plus;
		// }
		// adaptive_damping_x = fuzzy.fuzzy_damping_i_cos(intention_x, cos_x, intention_area, cos_area,
		// 											   RULE2, Damping_Area2);
		// adaptive_damping_y = fuzzy.fuzzy_damping_i_cos(intention_y, cos_y, intention_area, cos_area,
		// 											   RULE2, Damping_Area2);
		//自适应阻尼
		B_matrix_66(0, 0) = adaptive_damping_x;
		// B_matrix_66(1, 1) = adaptive_damping_y;
		cout << F_matrix_61[0] << "  " << actual_TCP_speed[0] << "  ";
		cout << B_matrix_66(0, 0) << endl;
		//固定阻尼
		// B_matrix_66(0, 0) = 50;
		// B_matrix_66(1, 1) = 50;
#endif
		// cout << "test damping: " << adaptive_damping_x << "\t";

// x方向直线RL
#if FQL

		if (stop_flag)
		{
			if (abs(F_matrix_61[0]) > 0.2 && loop_times % 5 == 0)
			{
				stop_flag = false;
				learning_flag = true;
				start_looptime = loop_times;
				cout << "---------------------read start-------------------" << endl;
				Q_table.open("Q_record.txt", ios::in | ios::out);
				// eligibility.open("eligibility_record.txt", ios::in | ios::out);
				if (!Q_table.is_open())
					cout << "open failed 001!" << endl;
				else
					cout << "open success 001!" << endl;

				for (int i = 0; i < fuzzy_RL.Q_table.size(); i++)
				{
					Q_table.getline(buffer, 100);
					sscanf(buffer, "%lf %lf %lf %lf ", &fuzzy_RL.Q_table[i][0], &fuzzy_RL.Q_table[i][1], &fuzzy_RL.Q_table[i][2],
						   &fuzzy_RL.Q_table[i][3]);
				}
				Q_table.close();
				cout << "---------------------read finished-------------------" << loop_times << endl;
				//初始化Etable
				vector<vector<double>> testE(state_num, vector<double>(damping_Area.size(), 0.0));
				fuzzy_RL.eligibility = testE;
			}
		}

		if (learning_flag) //启动RL
		{
			reward += -abs(jerk_x * jerk_x); // jerk 2-范数的积分
			// x_e = (actual_TCP_pose[0] - xout[0]);
			// reward += q1 * (F_matrix_61[0]) * (F_matrix_61[0]) + q2 * x_e * x_e;
			temp_reward = reward;
			total_reward += reward; //记录用
			if (loop_times % 5 == 0)
			{
				// FQL部分
				if (loop_times != start_looptime) //第一次不更新
				{
					phi = fuzzy_RL.fuzzy_inference(state);
					Q_star = fuzzy_RL.cal_Qvalue(phi);
					fuzzy_RL.update_Qtable(Q_val, Q_star, reward);
					reward = 0.0;
				}

				phi = fuzzy_RL.fuzzy_inference(state);
				B_matrix_66(0, 0) = fuzzy_RL.select_action(phi); //储存了phi_current下的local action
				Q_val = fuzzy_RL.cal_Qvalue(phi);
				fuzzy_RL.update_Etable(phi);
			}

			if (abs(F_matrix_61[0]) < 0.2)
				record_flag++;

			if (record_flag >= 10)
			{
				// Q表初始化
				// vector<vector<double>> testQ(state_num, vector<double>(damping_Area.size(), -3.0));
				// for (int i = 0; i < testQ.size(); i++)
				// {
				// 	testQ[i][0] = -1.0;
				// }
				// fuzzy_RL.Q_table = testQ;
				cout << "------------------record start-------------------" << endl;
				//每次拖动结束记录Q表
				Q_table.open("Q_record.txt", ios::in | ios::out | ios::trunc);
				// eligibility.open("eligibility_record.txt", ios::in | ios::out | ios::trunc);
				if (!Q_table.is_open())
					cout << "open failed 002!" << endl;
				else
					cout << "open success 002!" << endl;

				for (int i = 0; i < fuzzy_RL.Q_table.size(); i++)
				{
					for (int j = 0; j < fuzzy_RL.Q_table[i].size(); j++)
					{
						Q_table << fuzzy_RL.Q_table[i][j] << " ";
					}
					Q_table << endl;
				}
				Q_table.close();
				end_looptime = loop_times;
				cout << "------------------record finished-------------------" << loop_times << "\t" << total_reward << endl;
				learning_flag = false;
				// stop_flag = true;
				// back_flag = true;
				record_flag = 0;
			}
		}

#endif

		//安全限位
		if (abs(actual_TCP_pose[0] - 0.2245) > 0.4)
		{
			return 0;
		}

		// 	cout << " force(x方向):" << F_matrix_61[0] << endl;
		// cout << "acc: " << current_pose_error(1) << "\t" << actual_TCP_pose[0] << "\t";
		// cout << "dotF_matrix_61: " << dotF_matrix_61 << "\t";
		// cout << " force(x方向): " << F_matrix_61[0] << "\t";
		// cout << "speed:" << speed_x << "\t";
		// cout << "damping: " << B_matrix_66(0, 0) << "\t";

		// cout << temp_reward << "\t";
		// cout << total_reward << "\t";

		// cout << "jerk:" << jerk_x << "\t";
		// cout << "x: " << actual_TCP_pose[0] << "\t";
		// cout << endl;

		//记录循环次数与时间（1-2）
		fprintf(recordFile, "%d %ld ", loop_times, d_time);
		//记录发送笛卡尔位置（3-18）
		fprintf(recordFile, "%lf %lf %lf %lf %lf %lf ", xout[0], xout[1], xout[2], xout[3], xout[4], xout[5]);
		//记录力信息（9-14）
		fprintf(recordFile, "%6lf %6lf %6lf %6lf %6lf %6lf ", F_matrix_61[0], F_matrix_61[1], F_matrix_61[2], F_matrix_61[3], F_matrix_61[4], F_matrix_61[5]);
		//记录当前笛卡尔位置（15-20）
		fprintf(recordFile, "%lf %lf %lf %lf %lf %lf ", actual_TCP_pose[0], actual_TCP_pose[1], actual_TCP_pose[2], actual_TCP_pose[3], actual_TCP_pose[4], actual_TCP_pose[5]);
		//记录差分之后的速度与加速度（21-29）
		fprintf(recordFile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf ", speed_x, speed_y, speed_z, accelerate_x, accelerate_y, accelerate_z, jerk_x, jerk_y, jerk_z);
		//记录返回力信息（30-35）
		fprintf(recordFile, "%6lf %6lf %6lf %6lf %6lf %6lf ", F_matrix_back[0], F_matrix_back[1], F_matrix_back[2], F_matrix_back[3], F_matrix_back[4], F_matrix_back[5]);
		//记录当前笛卡尔速度（36-41）
		fprintf(recordFile, "%lf %lf %lf %lf %lf %lf ", actual_TCP_speed[0], actual_TCP_speed[1], actual_TCP_speed[2], actual_TCP_speed[3], actual_TCP_speed[4], actual_TCP_speed[5]);
		//记录学习相关量(42-44)
		fprintf(recordFile, "%lf %lf %lf ", B_matrix_66(0, 0), B_matrix_66(1, 1), B_matrix_66(2, 2));
		fprintf(recordFile, "%lf %lf %lf %lf ", intention_x, intention_y, cos_x, cos_y);
		// temp_reward, total_reward);
		// // FQL开始和结束的时间（43 44）
		// fprintf(recordFile, "%d %d %ld", start_looptime, end_looptime, adcontrol_time);
		//换行
		fprintf(recordFile, "\n");

		//循环结束后的变量赋值
		loop_times++;
		position_index++;
	}

	//	}

	return 0;
}