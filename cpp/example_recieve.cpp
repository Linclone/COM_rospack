#include "serial_driver.h"
#include <random>

/*  
  接收串口数据流程：
  1、打开串口
  2、编写回调函数
  3、开启接收线程
  注：
    编译要带上-lthread：
    g++ -o recieve example_recieve.cpp -lpthread
*/


int  error=0;
int recieve_num = 1;  // 调试用，统计收包数量

/* 
  用户自定义回调函数的内容

  功能：回调函数，处理接收的数据
  传入参数：数据起始位置及长度
  返回值：无
*/
void rxdone_callback(uint8_t *data_ptr,uint16_t data_length)
{
  /*
  for (int i = 0; i < rx_data_length-2; ++i)
    printf("%02X ",data_ptr[i]);
  std::cout << std::endl;
  */
  if(data_length!=560){
    printf("\n warning!!!! \n");
    error++;
  }
  printf("%d  %d  --%d\n",recieve_num++,data_length,error);
}



int main(void)
{
  // 打开串口
  int fd;
  char serial[] = "/dev/ttyACM0";      // 查看串口命令   ls -l /dev/ttyACM*
  fd = open_serial(serial);

  recieve_serial(fd, rxdone_callback);   // 开启接收线程

  while(1){}
  
}