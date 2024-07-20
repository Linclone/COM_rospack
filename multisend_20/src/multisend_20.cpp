#include "multisend_20/serial_driver.h"
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <algorithm>
#include <functional>
#include "geometry_msgs/PoseStamped.h"

#define Num_UAV  20
#define len  8 

////////////////////////////////////////////////////////////////////////
//--------- version: 1.0
// In this version, only topic "/pose" is being sent to UAV 
// support number of 20 UAVs 
// orientation: x^2+y^2+z^2+w^2=1
char serial[] = "/dev/ttyACM0";      // 查看串口命令   ls -l /dev/ttyACM*
// topics:
char uav0[] = "/vrpn_client_node/Fast/pose"; 
char uav1[] = "/vrpn_client_node/Fast/pose";
char uav2[] = "/vrpn_client_node/Fast/pose";
char uav3[] = "/vrpn_client_node/Fast/pose";
char uav4[] = "/vrpn_client_node/Rigid4/pose";
char uav5[] = "/vrpn_client_node/Rigid5/pose";
char uav6[] = "/vrpn_client_node/Rigid6/pose";
char uav7[] = "/vrpn_client_node/Rigid7/pose";
char uav8[] = "/vrpn_client_node/Rigid8/pose";
char uav9[] = "/vrpn_client_node/Rigid9/pose";
char uav10[] = "/vrpn_client_node/Rigid10/pose";
char uav11[] = "/vrpn_client_node/Rigid11/pose";
char uav12[] = "/vrpn_client_node/Rigid12/pose";
char uav13[] = "/vrpn_client_node/Rigid13/pose";
char uav14[] = "/vrpn_client_node/Rigid14/pose";
char uav15[] = "/vrpn_client_node/Rigid15/pose";
char uav16[] = "/vrpn_client_node/Rigid16/pose";
char uav17[] = "/vrpn_client_node/Rigid17/pose";
char uav18[] = "/vrpn_client_node/Rigid18/pose";
char uav19[] = "/vrpn_client_node/Rigid19/pose";

double f = 100 ;  //Hz

//////////////////////////////////////////////////////////////////////


int fd = open_serial(serial);
double t = 1.0/f ;

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
int flag=0;
std::vector<uint8_t> encodeData_pose0, encodeData_pose1, encodeData_pose2, encodeData_pose3, encodeData_pose4, encodeData_pose5, encodeData_pose6, encodeData_pose7, encodeData_pose8, encodeData_pose9;
std::vector<uint8_t> encodeData_pose10, encodeData_pose11, encodeData_pose12, encodeData_pose13, encodeData_pose14, encodeData_pose15, encodeData_pose16,  encodeData_pose17, encodeData_pose18, encodeData_pose19;
std::vector<uint8_t> encodeData_frameid;
uint8_t encodeData[1060]={0};

void send_callback( const ros::TimerEvent & ){
  // while(flag_write){}//do not allowed to send when updating
  // flag_send=1;
  uint8_t * ptr = encodeData;               // 指向数据起始位置的指针
  int i=1060;
  send_serial(fd, ptr, i, Num_UAV);   // 发送
  ROS_INFO(">--Pose send--%d bytes\n",i);
  // flag_send=0;
}



void pose_callback0(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i = 0; //shift amount

  encodeData_frameid.clear();
	encodeData_frameid.insert(encodeData_frameid.end(),msg.header.frame_id.begin(), msg.header.frame_id.end());
  encodeData_frameid.push_back ( 0 );  //"\0"
  encodeData_pose0.clear();
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose0.insert(encodeData_pose0.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose0.insert(encodeData_pose0.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose0.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j] = encodeData_pose0[j];
  }

  for(int j = 0;  ; j ++){
    encodeData[j+1045] = encodeData_frameid[j];
    if (encodeData[j+1045]==0)break;
  }
  // flag_write=0;

    // ROS_INFO("pose:\n");
  // ROS_INFO("position:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",encodeData2[0],encodeData2[1],encodeData2[2]);
  // ROS_INFO("orientation:\nx:%0.6f\ny:%0.6f\nz:%0.6f\n",encodeData2[3],encodeData2[4],encodeData2[5]);
  //send_serial(fd, ptr, len*i, Num_UAV);   // 发送
  //ROS_INFO(">--Pose send--%d bytes\n",i);
}

void pose_callback1(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =56; //shift amount
  encodeData_pose1.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose1.insert(encodeData_pose1.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose1.insert(encodeData_pose1.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose1.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose1[j];
  }
}

void pose_callback2(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =108; //shift amount
  encodeData_pose2.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose2.insert(encodeData_pose2.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose2.insert(encodeData_pose2.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose2.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose2[j];
  }
}

void pose_callback3(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =160; //shift amount
  encodeData_pose3.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose3.insert(encodeData_pose3.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose3.insert(encodeData_pose3.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose3.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose3[j];
  }
}

void pose_callback4(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =212; //shift amount
  encodeData_pose4.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose4.insert(encodeData_pose4.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose4.insert(encodeData_pose4.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose4.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose4[j];
  }
}

void pose_callback5(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =264; //shift amount
  encodeData_pose5.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose5.insert(encodeData_pose5.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose5.insert(encodeData_pose5.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose5.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose5[j];
  }
}

void pose_callback6(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =316; //shift amount
  encodeData_pose6.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose6.insert(encodeData_pose6.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose6.insert(encodeData_pose6.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose6.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose6[j];
  }
}

void pose_callback7(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =368; //shift amount
  encodeData_pose7.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose7.insert(encodeData_pose7.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose7.insert(encodeData_pose7.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose7.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose7[j];
  }
}

void pose_callback8(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =420; //shift amount
  encodeData_pose8.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose8.insert(encodeData_pose8.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose8.insert(encodeData_pose8.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose8.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose8[j];
  }
}

void pose_callback9(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =472; //shift amount
  encodeData_pose9.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose9.insert(encodeData_pose9.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose9.insert(encodeData_pose9.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose9.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose9[j];
  }
}

void pose_callback10(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =524; //shift amount
  encodeData_pose10.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose10.insert(encodeData_pose10.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose10.insert(encodeData_pose10.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose10.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose10[j];
  }
}

void pose_callback11(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =576; //shift amount
  encodeData_pose11.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose11.insert(encodeData_pose11.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose11.insert(encodeData_pose11.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose11.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose11[j];
  }
}

void pose_callback12(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =628; //shift amount
  encodeData_pose12.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose12.insert(encodeData_pose12.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose12.insert(encodeData_pose12.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose12.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose12[j];
  }
}

void pose_callback13(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =680; //shift amount
  encodeData_pose13.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose13.insert(encodeData_pose13.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose13.insert(encodeData_pose13.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose13.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose13[j];
  }
}

void pose_callback14(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =732; //shift amount
  encodeData_pose14.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose14.insert(encodeData_pose14.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose14.insert(encodeData_pose14.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose14.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose14[j];
  }
}

void pose_callback15(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =784; //shift amount
  encodeData_pose15.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose15.insert(encodeData_pose15.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose15.insert(encodeData_pose15.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose15.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose15[j];
  }
}

void pose_callback16(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =836; //shift amount
  encodeData_pose16.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose16.insert(encodeData_pose16.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose16.insert(encodeData_pose16.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose16.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose16[j];
  }
}

void pose_callback17(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =888; //shift amount
  encodeData_pose17.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose17.insert(encodeData_pose17.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose17.insert(encodeData_pose17.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose17.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose17[j];
  }
}

void pose_callback18(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =940; //shift amount
  encodeData_pose18.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose18.insert(encodeData_pose18.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose18.insert(encodeData_pose18.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose18.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose18[j];
  }
}

void pose_callback19(const geometry_msgs::PoseStamped &msg) 
{
  // while(flag_send){}//do not allowed to update when sending
  // flag_write=1; 
  int i =992; //shift amount
  encodeData_pose19.clear();
  std::vector<int> int_temp;
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
  std::vector<uint8_t> encodeData2;
  EncodeDouble(double_temp, encodeData2);


  encodeData_pose19.insert(encodeData_pose19.end(), encodeData1.begin(), encodeData1.end());
  encodeData_pose19.insert(encodeData_pose19.end(), encodeData2.begin(), encodeData2.end());

  int end=encodeData_pose19.size();
  for(int j = 0; j < end; j ++){
    encodeData[i+j]=encodeData_pose19[j];
  }
}



int main(int argc,char **argv)
{
  ros::init(argc, argv, "multisend_20");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub[20];

  {
    pose_sub[0] = nh.subscribe(uav0, 10, pose_callback0 );
    pose_sub[1] = nh.subscribe(uav1, 10, pose_callback1 );
    pose_sub[2] = nh.subscribe(uav2, 10, pose_callback2 );
    pose_sub[3] = nh.subscribe(uav3, 10, pose_callback3 );
    pose_sub[4] = nh.subscribe(uav4, 10, pose_callback4 );
    pose_sub[5] = nh.subscribe(uav5, 10, pose_callback5 );
    pose_sub[6] = nh.subscribe(uav6, 10, pose_callback6 );
    pose_sub[7] = nh.subscribe(uav7, 10, pose_callback7 );
    pose_sub[8] = nh.subscribe(uav8, 10, pose_callback8 );
    pose_sub[9] = nh.subscribe(uav9, 10, pose_callback9 );
    pose_sub[10] = nh.subscribe(uav10, 10, pose_callback10 );
    pose_sub[11] = nh.subscribe(uav11, 10, pose_callback11 );
    pose_sub[12] = nh.subscribe(uav12, 10, pose_callback12 );
    pose_sub[13] = nh.subscribe(uav13, 10, pose_callback13 );
    pose_sub[14] = nh.subscribe(uav14, 10, pose_callback14 );
    pose_sub[15] = nh.subscribe(uav15, 10, pose_callback15 );
    pose_sub[16] = nh.subscribe(uav16, 10, pose_callback16 );
    pose_sub[17] = nh.subscribe(uav17, 10, pose_callback17 );
    pose_sub[18] = nh.subscribe(uav18, 10, pose_callback18 );
    pose_sub[19] = nh.subscribe(uav19, 10, pose_callback19 );
  }


  ros::Timer timer = nh. createTimer ( ros::Duration ( t ), send_callback);//60Hz
  ros::spin();
}