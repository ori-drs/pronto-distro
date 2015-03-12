// Selective ros2lcm translator for thor mang
// mfallon
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <atlas_msgs/ForceTorqueSensors.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/robot_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/microstrain/ins_t.hpp"
using namespace std;

class App{
public:
  App(ros::NodeHandle node_, bool send_ground_truth_);
  ~App();

private:
  bool send_ground_truth_; // publish control msgs to LCM
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Joints:
  ros::Subscriber  joint_states_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  void appendFootSensor(pronto::force_torque_t& msg_out);
  
  void l_foot_sensor_cb(const geometry_msgs::WrenchStampedConstPtr& msg);  
  ros::Subscriber l_foot_sensor_sub_;
  geometry_msgs::WrenchStamped l_foot_sensor_;
  void r_foot_sensor_cb(const geometry_msgs::WrenchStampedConstPtr& msg);  
  ros::Subscriber r_foot_sensor_sub_;
  geometry_msgs::WrenchStamped r_foot_sensor_;

  void imu_cb(const sensor_msgs::ImuConstPtr& msg);
  ros::Subscriber imu_sub_;
  sensor_msgs::Imu imu_msg_;

  double last_ins_quat_[4];
  int64_t last_joint_state_utime_;
  bool verbose_;
};

App::App(ros::NodeHandle node_, bool send_ground_truth_) :
    send_ground_truth_(send_ground_truth_), node_(node_){
  ROS_INFO("Initializing Translator");
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  joint_states_sub_ = node_.subscribe(string("/thor_mang/joint_states"), 100, &App::joint_states_cb,this);
  l_foot_sensor_sub_ = node_.subscribe(string("/thor_mang/l_foot_raw"), 100, &App::l_foot_sensor_cb,this);
  r_foot_sensor_sub_ = node_.subscribe(string("/thor_mang/r_foot_raw"), 100, &App::r_foot_sensor_cb,this);
  imu_sub_ = node_.subscribe(string("/thor_mang/pelvis_imu"), 100, &App::imu_cb,this);

  verbose_ = false;
};

App::~App()  {
}


void App::l_foot_sensor_cb(const geometry_msgs::WrenchStampedConstPtr& msg){
  l_foot_sensor_ = *msg;
}
void App::r_foot_sensor_cb(const geometry_msgs::WrenchStampedConstPtr& msg){
  r_foot_sensor_ = *msg;
}

void App::imu_cb(const sensor_msgs::ImuConstPtr& msg){
  imu_msg_ = *msg;

  microstrain::ins_t m;
  m.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  m.device_time = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  m.gyro[0] = msg->angular_velocity.x;
  m.gyro[1] = msg->angular_velocity.y;
  m.gyro[2] = msg->angular_velocity.z;
  m.mag[0]=0;
  m.mag[1]=0;
  m.mag[2]=0;
  m.accel[0] = msg->linear_acceleration.x;
  m.accel[1] = msg->linear_acceleration.y;
  m.accel[2] = msg->linear_acceleration.z;
  m.quat[0] = msg->orientation.w;
  m.quat[1] = msg->orientation.x;
  m.quat[2] = msg->orientation.y;
  m.quat[3] = msg->orientation.z;
  m.pressure = 0;
  m.rel_alt =0 ;
  lcm_publish_.publish("MICROSTRAIN_INS", &m);

  memcpy(last_ins_quat_, m.quat, 4*sizeof(double) );
}

void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){

  pronto::force_torque_t force_torque;
  appendFootSensor(force_torque);

  bool send_est_robot_state = true;
  if ( send_est_robot_state ){
    pronto::robot_state_t m;
    m.pose.translation.x =0;
    m.pose.translation.y =0;
    m.pose.translation.z =0.85;
    m.pose.rotation.w = last_ins_quat_[0];
    m.pose.rotation.x = last_ins_quat_[1];
    m.pose.rotation.y = last_ins_quat_[2];
    m.pose.rotation.z = last_ins_quat_[3];
    m.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
    m.num_joints =msg->name.size();
    m.joint_name = msg->name;

    for (size_t i = 0; i < msg->name.size(); i++)  {
      m.joint_position.push_back( msg->position[ i ] );
      m.joint_velocity.push_back( msg->velocity[ i ] );
      m.joint_effort.push_back( msg->effort[ i ] );
    }

    m.force_torque = force_torque;
    lcm_publish_.publish("EST_ROBOT_STATE", &m);

    bot_core::pose_t p;
    p.utime = m.utime;
    p.pos[0]=0; p.pos[1]=0; p.pos[2]=0.85;
    memcpy(p.orientation, last_ins_quat_, 4*sizeof(double) );
    lcm_publish_.publish("POSE_BODY", &p);

  }


  bool send_atlas_robot_state = true;
  if ( send_atlas_robot_state ){
    pronto::atlas_state_t m;
    m.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
    m.num_joints =msg->name.size();

    for (size_t i = 0; i < msg->name.size(); i++)  {
      m.joint_position.push_back( msg->position[ i ] );
      m.joint_velocity.push_back( msg->velocity[ i ] );
      m.joint_effort.push_back( msg->effort[ i ] );
    }

    m.force_torque = force_torque;
    lcm_publish_.publish("ATLAS_STATE", &m);
  }

  pronto::utime_t utime_msg;
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  utime_msg.utime = joint_utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg); 

  last_joint_state_utime_ = joint_utime;
}


void App::appendFootSensor(pronto::force_torque_t& msg_out){
  msg_out.l_foot_force_z  =  l_foot_sensor_.wrench.force.z;
  msg_out.l_foot_torque_x =  l_foot_sensor_.wrench.torque.x;
  msg_out.l_foot_torque_y =  l_foot_sensor_.wrench.torque.y;
  msg_out.r_foot_force_z  =  r_foot_sensor_.wrench.force.z;
  msg_out.r_foot_torque_x =  r_foot_sensor_.wrench.torque.x;
  msg_out.r_foot_torque_y =  r_foot_sensor_.wrench.torque.y;

  msg_out.l_hand_force[0] =  0;
  msg_out.l_hand_force[1] =  0;
  msg_out.l_hand_force[2] =  0;
  msg_out.l_hand_torque[0] = 0;
  msg_out.l_hand_torque[1] = 0;
  msg_out.l_hand_torque[2] = 0;
  msg_out.r_hand_force[0] =  0;
  msg_out.r_hand_force[1] =  0;
  msg_out.r_hand_force[2] =  0;
  msg_out.r_hand_torque[0] =  0;
  msg_out.r_hand_torque[1] =  0;
  msg_out.r_hand_torque[2] =  0;
}


int main(int argc, char **argv){
  bool send_ground_truth = false;  

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh, send_ground_truth);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("ROS2LCM Translator Ready");
  ros::spin();
  return 0;
}
