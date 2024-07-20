#include "multireceive/serial_driver.h"
#include <ros/ros.h>
#include <stdio.h>
#include<string>
#include <algorithm>
#include <functional>
#include <iostream>
#include <cmath>  
#include "geometry_msgs/PoseStamped.h"


/////////////////////////////////////////////////////////////
//--------- version: 1.1
//--------- only publish PoseStamped

int UAV_No = 3; // from 0~19
char serial[] = "/dev/ttyACM5";      // 查看串口命令   ls -l /dev/ttyACM*


/////////////////////////////////////////////////////////////



geometry_msgs::PoseStamped P_msg;

using namespace std;

double decode2double(uint8_t* data_ptr) {
	void* p = data_ptr;
	double* a = static_cast<double*>(p);
	return a[0];
} //change uint8_t data_ptr[8] into double

int decode2int(uint8_t* data_ptr) {
	void* p = data_ptr;
	int* a = static_cast<int*>(p);
	return a[0];
} //change uint8_t data_ptr[4] into int

int sh;//shift amount
int refer_nsec;
int sec;
int nsec;
double x,y,z,w;

ros::Publisher pose_pub;

void rxdone_callback(uint8_t *data_ptr,uint16_t data_length)
{  
  ROS_INFO("find message with %d Bytes", data_length);

  sec=decode2int(data_ptr);
  uint8_t *ptr_temp=data_ptr + 4;
  refer_nsec=decode2int(ptr_temp);


  ptr_temp=data_ptr + sh;
  nsec=decode2int(ptr_temp);

  int substract=nsec-refer_nsec;
  if(substract<-750000000)++sec;
  else if(substract>750000000)--sec;
      
  P_msg.header.stamp.sec=sec;
  P_msg.header.stamp.nsec=nsec;

  ptr_temp=ptr_temp+4;
  P_msg.pose.position.x=decode2double(ptr_temp);
  ptr_temp=ptr_temp+8;
  P_msg.pose.position.y=decode2double(ptr_temp);
  ptr_temp=ptr_temp+8;
  P_msg.pose.position.z=decode2double(ptr_temp);
  ptr_temp=ptr_temp+8;
  x=decode2double(ptr_temp);
  ptr_temp=ptr_temp+8;
  y=decode2double(ptr_temp);
  ptr_temp=ptr_temp+8;
  z=decode2double(ptr_temp);
  w=1-(x*x+y*y+z*z);
  if(w<=0)w=0.0;
  else w=sqrt(w);
  P_msg.pose.orientation.x=x;
  P_msg.pose.orientation.y=y;
  P_msg.pose.orientation.z=z;
  P_msg.pose.orientation.w=w;

P_msg.header.frame_id=(char*)(data_ptr +1045);

      ROS_INFO(">--Pose recieve--\n");
      if(data_length!=0)pose_pub.publish(P_msg);
}


int main(int argc, char **argv)
{
  sh = UAV_No * 52 + 4;
  ros::init(argc, argv, "multireceive");
  ros::NodeHandle nh;
  pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
  int fd;
  ROS_INFO("start to receive from %s", serial);
  fd = open_serial(serial);
  //ros::Rate loop_rate(50); //50Hz
  recieve_serial(fd, rxdone_callback);   // 开启接收线程
  
  while(1){}
}