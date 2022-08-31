#include "ATI_UDP.h"

// 构造函数
ATIUDP::ATIUDP()
{
  /*
  //1、打开网络库
  WORD wdVersion = MAKEWORD(2, 2);
  WSADATA wdSockMsg;
  int nRes = WSAStartup(wdVersion, &wdSockMsg);
  */
  // 1、创建socket
  this->socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->socketHandle == -1)
  {
    cout << "力传感器连接失败" << endl;
    exit(1);
  }
  //	2、绑定地址与端口号
  *(uint16 *)&this->request[0] = htons(0x1234);
  *(uint16 *)&this->request[2] = htons(SENSOR_COMMAND);
  *(uint32 *)&this->request[4] = htonl(NUM_SAMPLES);
  this->he = gethostbyname(SENSOR_ADDRESS);
  memcpy(&this->addr.sin_addr, this->he->h_addr_list[0], this->he->h_length);
  this->addr.sin_family = AF_INET;
  this->addr.sin_port = htons(SENSOR_PORT);
  //	3、连接socket
  cout << "connect" << endl;
  sensor_err = connect(this->socketHandle, (struct sockaddr *)&this->addr, sizeof(this->addr));
  if (sensor_err == -1)
  {
    cout << "sensor_err: " << sensor_err << endl;
    exit(2);
  }
}

// send
void ATIUDP::Send()
{
  send(this->socketHandle, (const char *)this->request, 8, 0);
}

// recv
void ATIUDP::Recv(void)
{
  recv(this->socketHandle, (char *)this->response, 36, 0);
  this->resp.rdt_sequence = ntohl(*(uint32 *)&this->response[0]);
  this->resp.ft_sequence = ntohl(*(uint32 *)&this->response[4]);
  this->resp.status = ntohl(*(uint32 *)&this->response[8]);
  for (int i = 0; i < 6; i++)
  {
    this->resp.FTData[i] = ntohl(*(int32 *)&this->response[12 + i * 4]);
  }
}

//获取力信息(传地址)
const int32 *ATIUDP::getForceData(void)
{
  return this->resp.FTData;
}

//获取力信息(传值)
int32 ATIUDP::getForceData(int i)
{
  return this->resp.FTData[i] - FTzero[i];
}

void ATIUDP::getForceData_process(void)
{
  for (int i = 0; i < 6; i++)
  {
    ati_f[i] = this->resp.FTData[i] - FTzero[i];
    //单位处理  N
    ati_f[i] = ati_f[i] / 1000000.0;

    //进行阈值处理
    // if (i < 3) {
    //   if (abs(ati_f[i]) < 0.05) {
    //     ati_f[i] = 0;
    //   }
    // } else {
    //   if (abs(ati_f[i]) < 0.01) {
    //     ati_f[i] = 0;
    //   }
    // }
  }
}

//获取状态信息
const uint32 ATIUDP::getStatus(void)
{
  return this->resp.status;
}
//发送接收函数，并处理力传感器数据，处理零漂，处理单位为N
void ATIUDP::send_Recv(void)
{
  this->Send();
  this->Recv();
  getForceData_process();
}

void ATIUDP::setZero(void)
{
  this->Send();
  this->Recv();
  for (int i = 0; i < 6; i++)
  {
    FTzero[i] = resp.FTData[i];
  }
}

// showForceData 显示数据
void ATIUDP::showForceData(void)
{
  // printf("Status: 0x%08x\n", this->resp.status);
  for (int i = 0; i < 6; i++)
  {
    printf("%s: %lf  ", this->AXES[i], (float)(this->resp.FTData[i] - FTzero[i]) / 1000000.0);
  }
  cout << endl;
}
