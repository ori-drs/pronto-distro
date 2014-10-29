// Selective ros2lcm translator
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

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <atlas_msgs/ForceTorqueSensors.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
//#include <lcmtypes/multisense.hpp>
//#include "lcmtypes/pronto/imu_t.hpp"

using namespace std;

class App{
public:
  App(ros::NodeHandle node_, bool send_ground_truth_);
  ~App();

private:
  bool send_ground_truth_; // publish control msgs to LCM
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Atlas Joints and FT sensor
  ros::Subscriber  joint_states_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void appendLimbSensor(pronto::force_torque_t& msg_out , atlas_msgs::ForceTorqueSensors msg_in);
  ros::Subscriber end_effector_sensors_sub_;  
  void end_effector_sensors_cb(const atlas_msgs::ForceTorqueSensorsConstPtr& msg);  
  atlas_msgs::ForceTorqueSensors end_effector_sensors_;
  
  // The position and orientation from BDI's own estimator:
  ros::Subscriber pose_bdi_sub_;  
  void pose_bdi_cb(const nav_msgs::OdometryConstPtr& msg);  

  // Multisense Joint Angles:
  ros::Subscriber  head_joint_states_sub_;  
  void head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  
  // Laser:
  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  ros::Subscriber rotating_scan_sub_;
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);  
};

App::App(ros::NodeHandle node_, bool send_ground_truth_) :
    send_ground_truth_(send_ground_truth_), node_(node_){
  ROS_INFO("Initializing Translator");
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Atlas Joints and FT sensor
  joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 100, &App::joint_states_cb,this);
  end_effector_sensors_sub_ = node_.subscribe(string("/atlas/force_torque_sensors"), 100, &App::end_effector_sensors_cb,this);

  // The position and orientation from BDI's own estimator (or GT from Gazebo):
  if (send_ground_truth_){
    pose_bdi_sub_ = node_.subscribe(string("/ground_truth_odom"), 100, &App::pose_bdi_cb,this);
  }else{
    pose_bdi_sub_ = node_.subscribe(string("/pose_bdi"), 100, &App::pose_bdi_cb,this);
  }

  // Multisense Joint Angles:
  head_joint_states_sub_ = node_.subscribe(string("/multisense_sl/joint_states"), 100, &App::head_joint_states_cb,this);

  // Laser:
  rotating_scan_sub_ = node_.subscribe(string("/multisense_sl/laser/scan"), 100, &App::rotating_scan_cb,this);
};

App::~App()  {
}


int scan_counter=0;
void App::rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  if (scan_counter%80 ==0){
    ROS_ERROR("LSCAN [%d]", scan_counter );
    //std::cout << "SCAN " << scan_counter << "\n";
  }  
  scan_counter++;
  send_lidar(msg, "SCAN");
}


void App::send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcm_publish_.publish(channel.c_str(), &scan_out);
}


void App::end_effector_sensors_cb(const atlas_msgs::ForceTorqueSensorsConstPtr& msg){
  end_effector_sensors_ = *msg;
}


int gt_counter =0;
void App::pose_bdi_cb(const nav_msgs::OdometryConstPtr& msg){
  if (gt_counter%200 ==0){
    ROS_ERROR("GRTH [%d]", gt_counter );
  }  
  gt_counter++;

  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  pose_msg.pos[0] = msg->pose.pose.position.x;
  pose_msg.pos[1] = msg->pose.pose.position.y;
  pose_msg.pos[2] = msg->pose.pose.position.z;
  pose_msg.orientation[0] =  msg->pose.pose.orientation.w;
  pose_msg.orientation[1] =  msg->pose.pose.orientation.x;
  pose_msg.orientation[2] =  msg->pose.pose.orientation.y;
  pose_msg.orientation[3] =  msg->pose.pose.orientation.z;
  
  lcm_publish_.publish("POSE_BDI", &pose_msg);  
  
}


void App::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  /*
  multisense::state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)  {
    msg_out.joint_name.push_back(msg->name[i]);      
    msg_out.joint_position.push_back(msg->position[i]);      
    msg_out.joint_velocity.push_back(msg->velocity[i]);
    msg_out.joint_effort.push_back( msg->effort[i] );
  }  
  msg_out.num_joints = msg->name.size();
  lcm_publish_.publish("MULTISENSE_STATE", &msg_out);  
  
  */
}


int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  if (js_counter%500 ==0){
    std::cout << "J ST " << js_counter << "\n";
  }  
  js_counter++;
  
  pronto::atlas_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  

  msg_out.joint_position.assign(msg->name.size() , std::numeric_limits<int>::min()  );
  msg_out.joint_velocity.assign(msg->name.size() , std::numeric_limits<int>::min()  );
  msg_out.joint_effort.assign(msg->name.size() , std::numeric_limits<int>::min()  );
  msg_out.num_joints = msg->name.size();
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)  {
    // msg_out.joint_name[i] = msg->name[i];
    msg_out.joint_position[i] = msg->position[i];      
    msg_out.joint_velocity[i] = msg->velocity[i];
    msg_out.joint_effort[i] = msg->effort[i];
  }  
  
  // Append FT sensor info
  pronto::force_torque_t force_torque;
  appendLimbSensor(force_torque, end_effector_sensors_);
  msg_out.force_torque = force_torque;
  lcm_publish_.publish("ATLAS_STATE", &msg_out);
  
  pronto::utime_t utime_msg;
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  utime_msg.utime = joint_utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg); 
}


void App::appendLimbSensor(pronto::force_torque_t& msg_out , atlas_msgs::ForceTorqueSensors msg_in){
  msg_out.l_foot_force_z =  msg_in.l_foot.force.z;
  msg_out.l_foot_torque_x = msg_in.l_foot.torque.x; 
  msg_out.l_foot_torque_y = msg_in.l_foot.torque.y;
  msg_out.r_foot_force_z =  msg_in.r_foot.force.z;
  msg_out.r_foot_torque_x = msg_in.r_foot.torque.x; 
  msg_out.r_foot_torque_y = msg_in.r_foot.torque.y;

  msg_out.l_hand_force[0] =  msg_in.l_hand.force.x;
  msg_out.l_hand_force[1] =  msg_in.l_hand.force.y;
  msg_out.l_hand_force[2] =  msg_in.l_hand.force.z;
  msg_out.l_hand_torque[0] =  msg_in.l_hand.torque.x;
  msg_out.l_hand_torque[1] =  msg_in.l_hand.torque.y;
  msg_out.l_hand_torque[2] =  msg_in.l_hand.torque.z;
  msg_out.r_hand_force[0] =  msg_in.r_hand.force.x;
  msg_out.r_hand_force[1] =  msg_in.r_hand.force.y;
  msg_out.r_hand_force[2] =  msg_in.r_hand.force.z;
  msg_out.r_hand_torque[0] =  msg_in.r_hand.torque.x;
  msg_out.r_hand_torque[1] =  msg_in.r_hand.torque.y;
  msg_out.r_hand_torque[2] =  msg_in.r_hand.torque.z;   
}


int main(int argc, char **argv){
  bool send_ground_truth = false;  

  /*
  std::string mode_argument;
  if (argc >= 2){
     mode_argument = argv[1];
  }else {
    ROS_ERROR("Need to have another argument in the launch file");
  }
  
  if (mode_argument.compare("vrc_cheats_enabled") == 0){
    send_ground_truth = true;
  }else if (mode_argument.compare("vrc_cheats_disabled") == 0){
    send_ground_truth = false;    
  }else {
    ROS_ERROR("mode_argument not understood");
    std::cout << mode_argument << " is not understood\n";
    exit(-1);
  }
  */

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh, send_ground_truth);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("ROS2LCM Translator Ready");
  ros::spin();
  return 0;
}
