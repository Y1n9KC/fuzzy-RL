#pragma once

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctime>
#include <iostream>

using namespace std;

#define SENSOR_PORT 49152 /* Port the Net F/T always uses */
#define SENSOR_COMMAND 2  /* Command code 2 starts streaming */
#define NUM_SAMPLES 1     /* Will send 1 sample before stopping */
#define SENSOR_ADDRESS "192.168.1.23"
/* Typedefs used so integer sizes are more explicit */
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

// --------------------atiUDP类---------------------
/** 使用方法及调用顺序
 * 1、确定传感器ip地址，创建ati力传感器类
 * 2、平稳时调用void setZero(void);设置零点
 * 3、void send_Recv(void); 发送请求和接收数据
 * 4、数据保存在ati_f[6];中
 */
class ATIUDP
{
private:
  int socketHandle;
  struct sockaddr_in addr;
  struct hostent *he;
  byte request[8];
  RESPONSE resp;
  byte response[36];
  int index;
  int sensor_err;
  const char *AXES[6] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
  int32 FTzero[6] = {0, 0, 0, 0, 0, 0}; //传感器零漂值

public:
  ATIUDP();                        //构造函数
  ~ATIUDP(){};                     //析构函数
  void Send(void);                 //发送请求
  void Recv(void);                 //接受数据
  void getForceData_process(void); //获取数据
  void setZero(void);              //设置零点
  void send_Recv(void);            //发送请求和接收数据

  const int32 *getForceData(void);
  int32 getForceData(int i);
  const uint32 getStatus(void); //获取状态

  void showForceData(void); //显示数据
  double ati_f[6];          //处理之后的力传感器数值
};
