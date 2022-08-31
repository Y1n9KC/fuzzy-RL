#include "robot_math.h"

//-------------------滤波器类中方法-----------------
/**初始化滤波器
 * f_cut: 截止频率
 * dT 运行周期
 */
Lp1Filter::Lp1Filter(float f_cut, float dT)
{
  this->RC = 1.0f / (2.0f * Pi * f_cut);
  this->dT = dT;
  this->k = this->dT / (this->RC + this->dT);
  this->state = 0;
}

Lp1Filter::~Lp1Filter()
{
}

/**应用低通滤波器
 * input 输入
 */
double Lp1Filter::run(const double &input)
{
  this->state = (1 - this->k) * this->state + this->k * input;
  return this->state;
}

//---------------------------滑动平均波器类--------------------------
SWA_Filter ::SWA_Filter(int window_size)
{
  this->window_size = window_size;
}

SWA_Filter ::~SWA_Filter() {}

double SWA_Filter ::run(const double &input)
{
  if (num < window_size)
  {
    record.push_back(input);
    num++;
    return input;
  }
  else
  {
    record.push_back(input);
    record.pop_front();
    if (record.size() == window_size)
    {
      double sum = 0;
      // for (list<double>::iterator it = record.begin(); it != record.end(); ++it) {
      //   sum += *it;
      // }
      for (const auto &i : record)
      {
        sum += i;
      }
      return sum / (window_size * 1.0);
    }
  }
}

//-------------------PID-----------------

PID::PID()
{
}

PID::~PID()
{
}

// pid初始化
void PID::PID_init(const double &Kp, const double &Ki, const double &Kd, const double &cal_time)
{
  cout << "PID_init begin \n";
  this->Kd = Kd;
  this->Ki = Ki;
  this->Kp = Kp;
  this->cal_time = cal_time;
  this->err = 0;
  this->last_err = 0;
  this->pid_actual = 0;
  this->integral = 0;
  this->Matrix_integral << 0, 0, 0, 0, 0, 0;
  cout << "PID_init end \n";
  void PID_show();
}
/**位置式pid
 * pid_desired: 期望
 * input pid输入
 */
double PID::PID_run(const double &pid_desired, const double &input)
{
  this->pid_desired = pid_desired;
  this->err = this->pid_desired - input;
  this->integral += this->err * this->cal_time;
  this->output = this->Kp * this->err + this->Ki * this->integral + this->Kd * (this->err - this->last_err) / this->cal_time;
  this->last_err = this->err;
  cout << "pid输出： " << this->output << "\t比例项：" << this->Kp * this->err << "\t积分项：" << this->Ki * this->integral << "\t微分项： " << this->Kd * (this->err - this->last_err) / this->cal_time << endl;
  return this->output;
}
//位置式pid传入矩阵
Eigen::Matrix<float, 6, 1> PID::PID_run(Eigen::Matrix<float, 6, 1> pid_desired, Eigen::Matrix<float, 6, 1> input)
{
  this->Matrix_pid_desired = pid_desired;
  this->Matrix_err = this->Matrix_pid_desired - input;
  this->Matrix_integral += this->Matrix_err * this->cal_time;
  this->Matrix_output = this->Kp * this->Matrix_err + this->Ki * this->Matrix_integral + this->Kd * (this->Matrix_last_err - this->Matrix_err) / this->cal_time;
  this->Matrix_last_err = this->Matrix_err;
  return this->Matrix_output;
}

/**增量式pid
 * pid_desired: 期望
 * input pid输入
 */
double PID::PID_increment_run(const double &pid_desired, const double &input)
{
}

void PID::PID_integral_setzero()
{
  this->integral = 0;
}

void PID::PID_show()
{
  cout << "比例系数： " << this->Kp << "\t"
       << "积分系数： " << this->Ki << "\t"
       << "微分系数： " << this->Kd << endl;
}

//---------------------------------五次多项式规划--------------------
//构造函数
Quintic_Polynomial_Plan::Quintic_Polynomial_Plan(double time_start,
                                                 double time_end,
                                                 double position_start,
                                                 double position_end,
                                                 double velocity_start,
                                                 double velocity_end,
                                                 double accelerate_start,
                                                 double accelerate_end,
                                                 int part)
{
  this->time_start = time_start;
  this->time_end = time_end;
  this->position_start = position_start;
  this->position_end = position_end;
  this->accelerate_start = accelerate_start;
  this->accelerate_end = accelerate_end;
  this->velocity_start = velocity_start;
  this->velocity_end = velocity_end;
  this->part = part;
  //进行多项式系数求解
  this->coefficient();
}
//构造函数（指定结束时间与位置,细分次数，其他参数默认为0）
Quintic_Polynomial_Plan::Quintic_Polynomial_Plan(double time_end, double position_end, double dt)
{
  this->time_start = 0;
  this->time_end = time_end;
  this->position_start = 0;
  this->position_end = position_end;
  this->accelerate_start = 0;
  this->accelerate_end = 0;
  this->velocity_start = 0;
  this->velocity_end = 0;
  this->part = 1 / dt * time_end;
  //进行多项式系数求解
  this->coefficient();
}
//构造函数（指定初始与结束所有参数）

Quintic_Polynomial_Plan::~Quintic_Polynomial_Plan() {}
//求得多项式系数
void Quintic_Polynomial_Plan::coefficient()
{
  double k0, k1, k2, k3, k4, k5;
  double T = this->time_end - this->time_start;
  double T2 = T * T;
  double q0 = this->position_start, q1 = this->position_end, a0 = this->accelerate_start,
         a1 = this->accelerate_end, v0 = this->velocity_start, v1 = this->velocity_end;
  double h = q1 - q0;
  k0 = q0;
  k1 = this->velocity_start;
  k2 = 0.5 * this->accelerate_start;
  k3 = (20.0 * h - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T2) / (2.0 * T * T2);
  k4 = (-30.0 * h + (14.0 * v1 + 16.0 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2.0 * T2 * T2);
  k5 = (12 * h - 6 * (v1 + v0) * T + (a1 - a0) * T2) / (2 * T2 * T2 * T);
  this->k0 = k0;
  this->k1 = k1;
  this->k2 = k2;
  this->k3 = k3;
  this->k4 = k4;
  this->k5 = k5;
  cout << "k0:  " << this->k0 << "k1:  " << this->k1 << "k2:  " << this->k2 << "k3:  " << this->k3
       << "k4:  " << this->k4 << "k5  " << this->k5 << endl;
}

double Quintic_Polynomial_Plan::move(int times)
{
  double yy;
  // double times1 = times;
  yy = this->k0 + this->k1 * (times / 1.0 / this->part - this->time_start) +
       this->k2 * pow((times / 1.0 / this->part - this->time_start), 2) +
       this->k3 * pow((times / 1.0 / this->part - this->time_start), 3) +
       this->k4 * pow((times / 1.0 / this->part - this->time_start), 4) +
       this->k5 * pow((times / 1.0 / this->part - this->time_start), 5);
  return yy;
}
std::vector<double> Quintic_Polynomial_Plan::getK()
{
  vector<double> K;
  K.push_back(this->k0);
  K.push_back(this->k1);
  K.push_back(this->k2);
  K.push_back(this->k3);
  K.push_back(this->k4);
  K.push_back(this->k5);
  return K;
}
