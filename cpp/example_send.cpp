#include "serial_driver.h"
#include <random>


/*  
 发送数据流程：
  1、打开串口
  2、在send_serial()函数中传入串口文件、数据起始地址、数据长度、无人机数量参数
  注：
    发送函数根据传入的地址和长度取数据，并不传入实参。处理数据的过程统一当做二进制处理，并不关心数据类型，应该是可以传任意类型的数据
*/

//  g++ -o send example_send.cpp -lpthread

/* 生成随机数据 */
std::vector<uint8_t> generateRandomData(size_t length) {
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister 19937 生成器
    std::uniform_int_distribution<uint8_t> dis(0, 255); // 均匀分布在 [0, 255]
 
    std::vector<uint8_t> randomData(length);
    for (size_t i = 0; i < length; ++i) {
        randomData[i] = dis(gen);
    }
    return randomData;
}


int main(void)
{
  int fd;
  char serial[] = "/dev/ttyACM0";      // 查看串口命令   ls -l /dev/ttyACM*
  fd = open_serial(serial);

  uint8_t * ptr;                // 指向数据起始位置的指针

  int length = 560;            // 数据包长度
  for(int i = 0;i<1000000;i++)     // 发送次数
  {
    std::vector<uint8_t> random_data = generateRandomData(length);   // 生成随机数据
    uint8_t * ptr = random_data.data();                              // 指向数据起始位置

    send_serial(fd, ptr, length, 5);   // 发送
    usleep(10000);                     // 10ms

    printf("%s %d \n", "发包个数" ,i+1);
  }

}