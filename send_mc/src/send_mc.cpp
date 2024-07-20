#include "send_mc/serial_driver.h"
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <algorithm>
#include <functional>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"


#define Num_UAV  1

  char serial[] = "/dev/ttyACM0";     
  int fd = open_serial(serial);

void EncodeDouble(const std::vector<double>& data, std::vector<uint8_t>& result)
{
    const double *dataPtr = data.data();
    uint32_t dataSize = data.size() * sizeof(double);
    std::size_t    current = 0;
 
    while (current + sizeof(uint8_t) <= dataSize)
    {
        uint8_t val;
        memcpy(&val, (uint8_t*)dataPtr + current, sizeof(val));
        current += sizeof(val);
        result.push_back(val);
    }
}

void Encode_int32(const std::vector<int>& data, std::vector<uint8_t>& result)
{
    const int *dataPtr = data.data();
    uint32_t dataSize = data.size() * sizeof(int);
    std::size_t    current = 0;
 
    while (current + sizeof(uint8_t) <= dataSize)
    {
        uint8_t val;
        memcpy(&val, (uint8_t*)dataPtr + current, sizeof(val));
        current += sizeof(val);
        result.push_back(val);
    }
}


int flag_write=0; 
int flag_send=0;
int flag=1;
std::vector<uint8_t> encodeData_pose, encodeData_twist, encodeData_accel;


void send_callback( const ros::TimerEvent & ){
  while(flag_write){}//do not allowed to send when updating
  flag_send=1;
  uint8_t * ptr;                // 指向数据起始位置的指针
  int i=0;
  switch(flag){
  case 1:
    i=encodeData_pose.size();
    ptr=encodeData_pose.data();
    send_serial(fd, ptr, i, Num_UAV);   // 发送
    ROS_INFO(">--Pose send--%d bytes\n",i);
    flag++;
    break;
  case 2:
    i=encodeData_twist.size();
    ptr=encodeData_twist.data();
    send_serial(fd, ptr, i, Num_UAV);   // 发送
    ROS_INFO(">--Twist send--%d bytes\n",i);
    flag++;
    break;
    case 3:
    i=encodeData_accel.size();
    ptr=encodeData_accel.data();
    send_serial(fd, ptr, i, Num_UAV);   // 发送
    ROS_INFO(">--Accel send--%d bytes\n",i);
    flag=1;
    break;
  }
  flag_send=0;
}



void pose_callback(const geometry_msgs::PoseStamped &msg) 
{
  while(flag_send){}//do not allowed to update when sending
  flag_write=1; 
  encodeData_pose.clear();
  int i=0;
  // i=encodeData_pose.size();
  // ROS_INFO("/////%d",i);
  encodeData_pose.assign ( 1,1 ); // Pose
	encodeData_pose.insert(encodeData_pose.end(),msg.header.frame_id.begin(), msg.header.frame_id.end());
  encodeData_pose.push_back ( 0 );  //"\0"
  std::vector<int> int_temp;
  int_temp.push_back(msg.header.stamp.sec);
  int_temp.push_back(msg.header.stamp.nsec);
  std::vector<uint8_t> encodeData1;
  Encode_int32(int_temp, encodeData1);
  std::vector<double>  double_temp ;
  uint8_t * ptr;                // 指向数据起始位置的指针
  double_temp.push_back (  msg.pose.position.x);
  double_temp.push_back ( msg.pose.position.y);
  double_temp.push_back ( msg.pose.position.z);
  double_temp.push_back ( msg.pose.orientation.x);
  double_temp.push_back (  msg.pose.orientation.y);
  double_temp.push_back (  msg.pose.orientation.z);
  double_temp.push_back (  msg.pose.orientation.w);
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose.insert(encodeData_pose.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose.insert(encodeData_pose.end(), encodeData2.begin(), encodeData2.end());

  flag_write=0;

    // ROS_INFO("pose:\n");
  // ROS_INFO("position:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",encodeData2[0],encodeData2[1],encodeData2[2]);
  // ROS_INFO("orientation:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",encodeData2[3],encodeData2[4],encodeData2[5]);
  //send_serial(fd, ptr, len*i, Num_UAV);   // 发送
  //ROS_INFO(">--Pose send--%d bytes\n",i);
}

void twist_callback(const geometry_msgs::TwistStamped& msg) 
{
  while(flag_send){}
  flag_write=1; //do not allowed to send during updating
  encodeData_twist.clear();
  encodeData_twist.assign ( 1,2 ); // Twist
	encodeData_twist.insert(encodeData_twist.end(),msg.header.frame_id.begin(), msg.header.frame_id.end());
  encodeData_twist.push_back ( 0 );  //"\0"
  //ROS_INFO("%s\n", encodeData_twist.data());
  std::vector<int> int_temp;
  int_temp.push_back(msg.header.stamp.sec);
  int_temp.push_back(msg.header.stamp.nsec);
  std::vector<uint8_t> encodeData1;
  Encode_int32(int_temp, encodeData1);

  std::vector<double>  double_temp ;
  uint8_t * ptr;                // 指向数据起始位置的指针
  double_temp.push_back ( msg.twist.linear.x);
  double_temp.push_back ( msg.twist.linear.y);
  double_temp.push_back ( msg.twist.linear.z);
  double_temp.push_back (  msg.twist.angular.x);
  double_temp.push_back ( msg.twist.angular.y);
  double_temp.push_back ( msg.twist.angular.z);
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_twist.insert(encodeData_twist.end(), encodeData1.begin(), encodeData1.end());
  encodeData_twist.insert(encodeData_twist.end(), encodeData2.begin(), encodeData2.end());

  flag_write=0;

  // ROS_INFO("twist:\n");
  // ROS_INFO("linear:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",temp[0],temp[1],temp[2]);
  // ROS_INFO("angular:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",temp[3],temp[4],temp[5]);
  //send_serial(fd, ptr, len*i, Num_UAV);   // 发送
  //ROS_INFO("--Twist send\n");
}

void accel_callback(const geometry_msgs::TwistStamped& msg) 
{
  while(flag_send){}
  flag_write=1; //do not allowed to send during updating
  encodeData_accel.clear();
  encodeData_accel.assign ( 1,3 ); // Accel
	encodeData_accel.insert(encodeData_accel.end(),msg.header.frame_id.begin(), msg.header.frame_id.end());
  encodeData_accel.push_back ( 0 );  //"\0"
  //ROS_INFO("%s\n", encodeData_accel.data());
  std::vector<int> int_temp;
  int_temp.push_back(msg.header.stamp.sec);
  int_temp.push_back(msg.header.stamp.nsec);
  std::vector<uint8_t> encodeData1;
  Encode_int32(int_temp, encodeData1);

  std::vector<double>  double_temp ;
  uint8_t * ptr;                // 指向数据起始位置的指针
  double_temp.push_back ( msg.twist.linear.x);
  double_temp.push_back ( msg.twist.linear.y);
  double_temp.push_back ( msg.twist.linear.z);
  double_temp.push_back (  msg.twist.angular.x);
  double_temp.push_back ( msg.twist.angular.y);
  double_temp.push_back ( msg.twist.angular.z);
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_accel.insert(encodeData_accel.end(), encodeData1.begin(), encodeData1.end());
  encodeData_accel.insert(encodeData_accel.end(), encodeData2.begin(), encodeData2.end());

  flag_write=0;


  // ROS_INFO("accel:\n");
  // ROS_INFO("linear:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",temp[0],temp[1],temp[2]);
  // ROS_INFO("angular:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",temp[3],temp[4],temp[5]);
  // send_serial(fd, ptr, len*i, Num_UAV);   // 发送
  // ROS_INFO("--Accel send\n");
}



int main(int argc,char **argv)
{
  ros::init(argc, argv, "send_mc");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub, twist_sub, accel_sub;
  pose_sub = nh.subscribe("/vrpn_client_node/Rigid0/pose", 10, pose_callback );
  twist_sub = nh.subscribe("/vrpn_client_node/Rigid0/twist", 10, twist_callback );
  accel_sub = nh.subscribe("/vrpn_client_node/Rigid0/accel", 10, accel_callback );
  ros::Timer timer = nh. createTimer ( ros::Duration ( 0.003), send_callback);//100Hz
  ros::spin();
}