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

#include <Eigen/Dense>

#include <atlas_hardware_interface/AtlasPrefilter.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"

#include "lcmtypes/pronto/multisense_state_t.hpp"
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
  
  void data_cb(const atlas_hardware_interface::AtlasPrefilterConstPtr& msg);
  ros::Subscriber data_sub_;
};

App::App(ros::NodeHandle node_, bool send_ground_truth_) :
    send_ground_truth_(send_ground_truth_), node_(node_){
  ROS_INFO("Initializing Translator");
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  data_sub_ = node_.subscribe(string("/atlas_hardware/data/prefilter"), 100, &App::data_cb,this);
};

App::~App()  {
}


int js_counter =0;
void App::data_cb(const atlas_hardware_interface::AtlasPrefilterConstPtr& msg){
  if (js_counter%500 ==0){
    std::cout << "J ST " << js_counter << "\n";
  }  
  js_counter++;

  int64_t utime = (int64_t) floor(msg->filtered_imu.header.stamp.toNSec()/1000);

  // 1....................................
  pronto::atlas_behavior_t msg_out;
  msg_out.utime = utime;
  msg_out.behavior = (int) msg->current_behavior.state;
  lcm_publish_.publish("ATLAS_BEHAVIOR", &msg_out);

  
  // 2....................................
  bot_core::pose_t pose_msg;
  pose_msg.utime = utime;

  pose_msg.pos[0] = msg->pos_est.position.x;
  pose_msg.pos[1] = msg->pos_est.position.y;
  pose_msg.pos[2] = msg->pos_est.position.z;
  // what about orientation in imu msg?
  pose_msg.orientation[0] =  msg->filtered_imu.orientation.w;
  pose_msg.orientation[1] =  msg->filtered_imu.orientation.x;
  pose_msg.orientation[2] =  msg->filtered_imu.orientation.y;
  pose_msg.orientation[3] =  msg->filtered_imu.orientation.z;

  // This transformation is NOT correct for Trooper
  // April 2014: added conversion to body frame - so that both rates are in body frame
  Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( msg->filtered_imu.orientation.w, msg->filtered_imu.orientation.x,
                                                           msg->filtered_imu.orientation.y, msg->filtered_imu.orientation.z ));
  Eigen::Vector3d lin_body_vel  = R*Eigen::Vector3d ( msg->pos_est.velocity.x, msg->pos_est.velocity.y,
                                                      msg->pos_est.velocity.z );
  pose_msg.vel[0] = lin_body_vel[0];
  pose_msg.vel[1] = lin_body_vel[1];
  pose_msg.vel[2] = lin_body_vel[2];

  // this is the body frame rate
  pose_msg.rotation_rate[0] = msg->filtered_imu.angular_velocity.x;
  pose_msg.rotation_rate[1] = msg->filtered_imu.angular_velocity.y;
  pose_msg.rotation_rate[2] = msg->filtered_imu.angular_velocity.z;
  
  // Frame?
  pose_msg.accel[0] = msg->filtered_imu.linear_acceleration.x;
  pose_msg.accel[1] = msg->filtered_imu.linear_acceleration.y;
  pose_msg.accel[2] = msg->filtered_imu.linear_acceleration.z;

  lcm_publish_.publish("POSE_BDI", &pose_msg);   
  lcm_publish_.publish("POSE_BODY", &pose_msg);    // for now

  
  // 3..........................................
  pronto::atlas_state_t js_out;
  js_out.utime = (int64_t) utime;
  int n_joints = 28;
  js_out.joint_position.assign(n_joints , std::numeric_limits<int>::min()  );
  js_out.joint_velocity.assign(n_joints , std::numeric_limits<int>::min()  );
  js_out.joint_effort.assign(n_joints , std::numeric_limits<int>::min()  );
  js_out.num_joints = n_joints;

  for (std::vector<int>::size_type i = 0; i < n_joints; i++)  {
    js_out.joint_position[i] = msg->j[i].q;
    js_out.joint_velocity[i] = msg->j[i].qd;
    js_out.joint_effort[i]   = msg->j[i].f;
  }

  // Append FT sensor info
  pronto::force_torque_t ft_out;
  ft_out.l_foot_force_z  =  msg->foot_sensors[0].force.z;
  ft_out.l_foot_torque_x =  msg->foot_sensors[0].torque.x;
  ft_out.l_foot_torque_y =  msg->foot_sensors[0].torque.y;
  ft_out.r_foot_force_z  =  msg->foot_sensors[1].force.z;
  ft_out.r_foot_torque_x =  msg->foot_sensors[1].torque.x;
  ft_out.r_foot_torque_y =  msg->foot_sensors[1].torque.y;
  ft_out.l_hand_force[0] =  0;
  ft_out.l_hand_force[1] =  0;
  ft_out.l_hand_force[2] =  0;
  ft_out.l_hand_torque[0] = 0;
  ft_out.l_hand_torque[1] = 0;
  ft_out.l_hand_torque[2] = 0;
  ft_out.r_hand_force[0] =  0;
  ft_out.r_hand_force[1] =  0;
  ft_out.r_hand_force[2] =  0;
  ft_out.r_hand_torque[0] =  0;
  ft_out.r_hand_torque[1] =  0;
  ft_out.r_hand_torque[2] =  0;
  js_out.force_torque = ft_out;

  lcm_publish_.publish("ATLAS_STATE", &js_out);


  // 4........................................................
  pronto::utime_t utime_msg;
  int64_t joint_utime = (int64_t) utime;
  utime_msg.utime = utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg); 

  
  // 5........................................................
  pronto::atlas_raw_imu_batch_t imu;
  imu.utime = (int64_t) utime;
  imu.num_packets = 15;
  for (size_t i=0; i < 15 ; i++){
    pronto::atlas_raw_imu_t raw;
    raw.utime = (int64_t) floor(msg->raw_imu[i].imu_timestamp.toNSec()/1000);
    raw.packet_count = msg->raw_imu[i].packet_count;
    raw.delta_rotation[0] = msg->raw_imu[i].da.x;
    raw.delta_rotation[1] = msg->raw_imu[i].da.y;
    raw.delta_rotation[2] = msg->raw_imu[i].da.z;
    
    raw.linear_acceleration[0] = msg->raw_imu[i].dd.x;
    raw.linear_acceleration[1] = msg->raw_imu[i].dd.y;
    raw.linear_acceleration[2] = msg->raw_imu[i].dd.z;
    imu.raw_imu.push_back( raw );
  }
  lcm_publish_.publish( ("ATLAS_IMU_BATCH") , &imu);

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
