
#include <iostream>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>



typedef void (*callback_fun_t)(uint8_t*, uint16_t);
callback_fun_t call_back = NULL;

int open_serial(char* serial);                                               // 打开串口                          
void send_serial(int fd, uint8_t *data_ptr,uint16_t data_size,int UAVNum);   // 串口发送
void recieve_serial(int fd, callback_fun_t callback_func);                   // 串口接受

void set_opt(int fd);                                            // 配置串口
void *serial_process(void* input);                              // 监听串口的线程函数
uint16_t crc16(uint16_t crc, uint8_t* data, uint32_t len);      // CRC计算
void byte_parse(uint8_t data);                                  // 帧格式解析状态机实现


uint8_t frame_id = 1;  // 帧序列，配合嵌入式端，从1开始

uint8_t submit_data[2000];    // 接收串口数据paylod的暂存区
uint16_t rx_data_length; 
int submit_data_ptr = 0;


uint16_t crc_paylaod; 
uint16_t crc_get; 
                                
typedef enum
{
    one,                // 判断 7E
    two,                // 判断 3F 
    three,              // 判断 00
		len_h,             // 负载长度高位
		len_l,             // .......低位
		num,               // 无人机数量
		ID,                // 帧序列
		data_recieve,      // 保存 payload和crc
}msghead_t;
msghead_t HeadState;               // 串口帧格式解析状态机



/* 串口帧格式解析状态机实现 */
void byte_parse(uint8_t data)
{
	switch( HeadState )
	{
		case one:
				if(data == 0x7E)          // 7E
					HeadState = two;
				else
					HeadState = one;
				break;		
		case two:
				if(data == 0x3F)          // 3F
					HeadState = three;
				else
					HeadState = one;
				break;	
		case three:
				if(data == 0x00)          // 00
					HeadState = len_h;
				else
					HeadState = one;
				break;	
		case len_h:    
        rx_data_length = (uint16_t)data << 8;                       // 保存负载长度高八位                               
				HeadState = len_l;
				break;		
		case len_l:  
        rx_data_length = rx_data_length | (uint16_t)data + 2;       // 保存低8位， 加上两位CRC的长度  
				HeadState = num;
				break;		
		case num:
				HeadState = ID;
				break;			
		case ID: 	
				HeadState = data_recieve;
			break;		
		case data_recieve:
				submit_data[submit_data_ptr++] = data;                // 开始保存有效数据
        if (submit_data_ptr == 1990)
        {
          HeadState = one;
          submit_data_ptr = 0;
        }
        
				if(submit_data_ptr == rx_data_length)                            // 数据包截取完毕
        {
          // 复位状态机
          HeadState = one;
          submit_data_ptr = 0;
          // 数据通过CRC，则执行回调函数  
          crc_paylaod = crc16(0xFFFF, submit_data, rx_data_length-2);
          crc_get = (uint16_t)submit_data[rx_data_length-2] << 8 | (uint16_t)submit_data[rx_data_length-1];
          if(crc_paylaod == crc_get)
          {
            call_back(submit_data, rx_data_length-2);
          }

  
        }
			break;
	} // switch( HeadState )
}


/*
  功能：打开串口
  传入参数：串口号字符串地址
  返回值：串口设备文件
*/
int open_serial(char* serial)
{
  int fd;
  fd = open(serial, O_RDWR | O_NOCTTY | O_NDELAY);
  
  if(fd == -1)
    throw std::runtime_error("串口打开失败！！！,请检查端口号是否正确、是否插入设备...");
  else
    set_opt(fd);                       // 设置参数
  return fd;
}

/* 配置串口 */
void set_opt(int fd)
{
  struct termios tty;
  bzero(&tty,sizeof(tty));

  cfsetispeed(&tty, B3000000);    // 波特率，USB协议，传输速度不受波特率影响
  cfsetospeed(&tty, B3000000);

  tty.c_cflag |= CS8;             // 数据位数，8位
  tty.c_cflag &= ~PARENB;         // 无校验位
  tty.c_cflag &= ~CSTOPB;         // 1位停止位

  tty.c_cc[VTIME]=0;
  tty.c_cc[VMIN]=0;               // 非阻塞式

  tcflush(fd,TCIFLUSH);           // 刷清输入队列，清除未处理数据

  // 激活配置
  if ((tcsetattr(fd, TCSANOW, &tty))!=0)
    throw std::runtime_error("serial set faild!!!");
  else
    printf("serial set done!\n");
}

/* 串口接收的线程函数 */
void *serial_process(void* input)
{
  int fd =*(int *)input;
  uint8_t buff[1];

  while(1)
  {
    if(read(fd,buff,1) > 0)  // 从串口读一个字节
      byte_parse(buff[0]);  
  }
}


/* 16位CRC校验 */
uint16_t crc16(uint16_t crc, uint8_t* data, uint32_t len) 
{        
    uint32_t i;
    for (i = 0; i < len; i++) 
    {        
      crc = ((crc >> 8) | (crc << 8)) & 0xFFFF;   
      crc ^= (data[i] & 0xFF);    
      crc ^= ((crc & 0xFF) >> 4);      
      crc ^= (crc << 12) & 0xFFFF;   
      crc ^= ((crc & 0xFF) << 5) & 0xFFFF;   
    }       
    return (crc & 0xFFFF);
}


/*
  功能：向串口发送数据
  传入参数：串口设备文件、数据起始地址、数据长度、无人机数量参数
  返回值：无
*/
void send_serial(int fd, uint8_t *data_ptr,uint16_t data_size,int UAVNum)
{

  uint16_t crc_lavue;
  uint16_t frame_data_length = data_size + 10;       // 加10位帧格式和CRC所占空间
  uint8_t frame_data[frame_data_length];             // 串口发送前的组帧缓冲区

  crc_lavue = crc16(0xFFFF, data_ptr, data_size);
  // 组帧
  frame_data[0] = 0x7E;
  frame_data[1] = 0x3F;
  frame_data[2] = 0x00;
  frame_data[3] = (data_size >> 8) & 0xFF;      // 负载长度高8位
  frame_data[4] = data_size;                    // 低8位
  frame_data[5] = UAVNum;
  frame_data[6] = frame_id;

  for(int i=0;i<data_size;i++)          // payload
     frame_data[7+i] = *data_ptr++;

  frame_data[7+data_size] = (crc_lavue >> 8) & 0xFF;    // CRC高8位
  frame_data[7+data_size+1] = crc_lavue;                // CRC低8位
  frame_data[7+data_size+2] = 0xAA;

  if(write(fd,frame_data, frame_data_length) == -1)  // 写串口
    throw std::runtime_error("串口写入失败！！");


  frame_id = frame_id+1;
  if(frame_id == 255)  
    frame_id = 1;

}

/*
  功能：开启监听串口的线程，接收串口数据，返回符合帧格式且通过CRC的数据
  传入参数：串口设备文件、回调函数
  返回值：无
*/
void recieve_serial(int fd, callback_fun_t callback_func)
{
    call_back = callback_func;  // 记录回调函数的位置
    pthread_t p_id;

    if(pthread_create(&p_id,NULL, serial_process, &fd) == 0)  // 开启接受线程
      printf("串口接收线程开启成功,等待串口数据ing..\n");
    else
      throw std::runtime_error("串口接收线程开启失败！！！");
}