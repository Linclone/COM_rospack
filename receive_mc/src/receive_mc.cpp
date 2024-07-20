#include "receive_mc/serial_driver.h"
#include <ros/ros.h>
#include <stdio.h>
#include<string>
#include <algorithm>
#include <functional>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"

geometry_msgs::PoseStamped P_msg;
geometry_msgs::TwistStamped T_msg; 
geometry_msgs::TwistStamped A_msg;

using namespace std;

void DecodeToDouble(const std::vector<uint8_t>& data, std::vector<double>& result)
{
    const uint8_t *dataPtr = data.data();
    uint32_t dataSize = data.size() * sizeof(uint8_t);
    std::size_t    current = 0;
 
    while (current + sizeof(double) <= dataSize)
    {
        double val;
        memcpy(&val, (uint8_t*)dataPtr + current, sizeof(val));
        current += sizeof(val);
        result.push_back(val);
    }
}


void DecodeToInt32(const std::vector<uint8_t>& data, std::vector<int>& result)
{
    const uint8_t *dataPtr = data.data();
    uint32_t dataSize = data.size() * sizeof(uint8_t);
    std::size_t    current = 0;
 
    while (current + sizeof(int) <= dataSize)
    {
        int val;
        memcpy(&val, (uint8_t*)dataPtr + current, sizeof(val));
        current += sizeof(val);
        result.push_back(val);
    }
}

ros::Publisher pose_pub,  twist_pub,  accel_pub;

// class stamps{
//   public:
//     ros::Publisher* pose_pub, * twist_pub, * accel_pub;
//     void rxdone_callback(uint8_t *data_ptr,uint16_t data_length);
// };



void rxdone_callback(uint8_t *data_ptr,uint16_t data_length)
{
  ROS_INFO("find message with %d Bytes", data_length);
  uint8_t *ptr_temp=data_ptr;
  ROS_INFO("cate is %d", (int)(ptr_temp[0]));
  std::vector<uint8_t> encodeData_int;
  std::vector<int> decodeData_int;
  std::vector<uint8_t> encodeData_double;
  std::vector<double> decodeData_double;
  const uint8_t x1=1;
  const uint8_t x2=2;
  const uint8_t x3=3;
  switch(ptr_temp[0]){
    case x1:
      //Pose
      ptr_temp++;
      P_msg.header.frame_id=(char*)(data_ptr);
      while(ptr_temp[0]!=0)ptr_temp++;
      ptr_temp++;

      for( int i=0 ; i<8 ; i++ ){
        encodeData_int.push_back(ptr_temp[0]);
        ptr_temp++;
      }
      DecodeToInt32(encodeData_int, decodeData_int);
      P_msg.header.stamp.sec=decodeData_int[0];
      P_msg.header.stamp.nsec=decodeData_int[1];
      

      for( int i=0 ; i<56 ; i++ ){
        encodeData_double.push_back(ptr_temp[0]);
        ptr_temp++;
      }
      DecodeToDouble(encodeData_double, decodeData_double);
      P_msg.pose.position.x=decodeData_double[0];
      P_msg.pose.position.y=decodeData_double[1];
      P_msg.pose.position.z=decodeData_double[2];
      P_msg.pose.orientation.x=decodeData_double[3];
      P_msg.pose.orientation.y=decodeData_double[4];
      P_msg.pose.orientation.z=decodeData_double[5];
      P_msg.pose.orientation.w=decodeData_double[6];
      ROS_INFO(">--Pose recieve--\n");
      if(data_length!=0)pose_pub.publish(P_msg);
      break;

    case x2:
      //Twist
      ptr_temp++;
      T_msg.header.frame_id=(char*)(data_ptr);
      while(ptr_temp[0]!=0)ptr_temp++;
      ptr_temp++;

      for( int i=0 ; i<8 ; i++ ){
        encodeData_int.push_back(ptr_temp[0]);
        ptr_temp++;
      }
      DecodeToInt32(encodeData_int, decodeData_int);
      T_msg.header.stamp.sec=decodeData_int[0];
      T_msg.header.stamp.nsec=decodeData_int[1];
      

      for( int i=0 ; i<48 ; i++ ){
        encodeData_double.push_back(ptr_temp[0]);
        ptr_temp++;
      }
      DecodeToDouble(encodeData_double, decodeData_double);
      T_msg.twist.linear.x=decodeData_double[0];
      T_msg.twist.linear.y=decodeData_double[1];
      T_msg.twist.linear.z=decodeData_double[2];
      T_msg.twist.angular.x=decodeData_double[3];
      T_msg.twist.angular.y=decodeData_double[4];
      T_msg.twist.angular.z=decodeData_double[5];
      ROS_INFO(">--Twist recieve--\n");
      if(data_length!=0)twist_pub.publish(T_msg);
      break;

      case x3:
      //Accel
      ptr_temp++;
      A_msg.header.frame_id=(char*)(data_ptr);
      while(ptr_temp[0]!=0)ptr_temp++;
      ptr_temp++;


      for( int i=0 ; i<8 ; i++ ){
        encodeData_int.push_back(ptr_temp[0]);
        ptr_temp++;
      }
      DecodeToInt32(encodeData_int, decodeData_int);
      A_msg.header.stamp.sec=decodeData_int[0];
      A_msg.header.stamp.nsec=decodeData_int[1];
      

      for( int i=0 ; i<48 ; i++ ){
        encodeData_double.push_back(ptr_temp[0]);
        ptr_temp++;
      }
      DecodeToDouble(encodeData_double, decodeData_double);
      A_msg.twist.linear.x=decodeData_double[0];
      A_msg.twist.linear.y=decodeData_double[1];
      A_msg.twist.linear.z=decodeData_double[2];
      A_msg.twist.angular.x=decodeData_double[3];
      A_msg.twist.angular.y=decodeData_double[4];
      A_msg.twist.angular.z=decodeData_double[5];
      ROS_INFO(">--Accel recieve--\n");
      if(data_length!=0)accel_pub.publish(A_msg);
      break;

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "receive_mc");
  ros::NodeHandle nh;
  pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
  twist_pub=nh.advertise<geometry_msgs::TwistStamped>("/twist", 1);
  accel_pub=nh.advertise<geometry_msgs::TwistStamped>("/accel", 1);
  int fd;
  char serial[] = "/dev/ttyACM1";      // 查看串口命令   ls -l /dev/ttyACM*
  ROS_INFO("start to receive from %s", serial);
  fd = open_serial(serial);
  //ros::Rate loop_rate(50); //50Hz
  recieve_serial(fd, rxdone_callback);   // 开启接收线程
  while(1){}


  // while(ros::ok()){

  //   pose_pub.publish(P_msg);
  //   twist_pub.publish(T_msg);
  //   accel_pub.publish(A_msg);
  //   ROS_INFO(">----published once----<\n");
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
}