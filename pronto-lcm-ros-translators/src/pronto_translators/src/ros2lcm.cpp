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
#include <trooper_mlc_msgs/FootSensor.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"

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
  
  // Atlas Joints and FT sensor
  ros::Subscriber  joint_states_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void appendFootSensor(pronto::force_torque_t& msg_out , trooper_mlc_msgs::FootSensor msg_in);
  
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

  // LM:
  void foot_sensor_cb(const trooper_mlc_msgs::FootSensorConstPtr& msg);  
  ros::Subscriber foot_sensor_sub_;
  trooper_mlc_msgs::FootSensor foot_sensor_;

  void imu_cb(const sensor_msgs::ImuConstPtr& msg);
  ros::Subscriber imu_sub_;
  sensor_msgs::Imu imu_msg_;

  //void imu_batch_cb(const sensor_msgs::ImuConstPtr& msg);
  //ros::Subscriber imu_batch_sub_;
  //sensor_msgs::Imu imu_batch_msg_;

  void sendMultisenseState(int64_t utime, float angle);
};

App::App(ros::NodeHandle node_, bool send_ground_truth_) :
    send_ground_truth_(send_ground_truth_), node_(node_){
  ROS_INFO("Initializing Translator");
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Atlas Joints and FT sensor
  joint_states_sub_ = node_.subscribe(string("joint_states"), 100, &App::joint_states_cb,this);

  // The position and orientation from BDI's own estimator (or GT from Gazebo):
  if (send_ground_truth_){
    pose_bdi_sub_ = node_.subscribe(string("/ground_truth_odom"), 100, &App::pose_bdi_cb,this);
  }else{
    pose_bdi_sub_ = node_.subscribe(string("/controller_mgr/bdi_odom"), 100, &App::pose_bdi_cb,this);
  }

  // Multisense Joint Angles:
  head_joint_states_sub_ = node_.subscribe(string("/multisense_sl/joint_states"), 100, &App::head_joint_states_cb,this);

  // Laser:
  rotating_scan_sub_ = node_.subscribe(string("/multisense_sl/laser/scan"), 100, &App::rotating_scan_cb,this);

  // LM:
  foot_sensor_sub_ = node_.subscribe(string("/foot_contact_service/foot_sensor"), 100, &App::foot_sensor_cb,this);
  imu_sub_ = node_.subscribe(string("/imu_publisher_service/imu"), 100, &App::imu_cb,this);
  //imu_batch_sub_ = node_.subscribe(string("/imu_publisher_service/raw_imu"), 100, &App::imu_batch_cb,this);

};

App::~App()  {
}


void App::foot_sensor_cb(const trooper_mlc_msgs::FootSensorConstPtr& msg){
  foot_sensor_ = *msg;
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
  // what about orientation in imu msg?
  pose_msg.orientation[0] =  msg->pose.pose.orientation.w;
  pose_msg.orientation[1] =  msg->pose.pose.orientation.x;
  pose_msg.orientation[2] =  msg->pose.pose.orientation.y;
  pose_msg.orientation[3] =  msg->pose.pose.orientation.z;

  // No BDI velocity in the logs

  // this is the body frame rate
  pose_msg.rotation_rate[0] = imu_msg_.angular_velocity.x;
  pose_msg.rotation_rate[1] = imu_msg_.angular_velocity.y;
  pose_msg.rotation_rate[2] = imu_msg_.angular_velocity.z;
  
  // Frame?
  pose_msg.accel[0] = imu_msg_.linear_acceleration.x;
  pose_msg.accel[1] = imu_msg_.linear_acceleration.y;
  pose_msg.accel[2] = imu_msg_.linear_acceleration.z;

  lcm_publish_.publish("POSE_BDI", &pose_msg);   
  lcm_publish_.publish("POSE_BODY", &pose_msg);    // for now
}

void App::imu_cb(const sensor_msgs::ImuConstPtr& msg){
  imu_msg_ = *msg;

  pronto::atlas_behavior_t bmsg;
  bmsg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  bmsg.behavior = 3; // 
  lcm_publish_.publish("ATLAS_BEHAVIOR", &bmsg);     
}


/*
void App::imu_batch_cb(int64_t robot_utime){
  ROS_ERROR("IMU Batch" );

  drc::atlas_raw_imu_batch_t imu;
  imu.utime = robot_utime;
  imu.num_packets = Atlas::NUM_RAW_IMU_PACKETS;
  for (size_t i=0; i < Atlas::NUM_RAW_IMU_PACKETS ; i++){
    
    //std::cout << i
    //  << " | " <<  s_data_from_robot.raw_imu[i].imu_timestamp
    //  << " | " <<  s_data_from_robot.raw_imu[i].packet_count
    //  << " | " <<  s_data_from_robot.raw_imu[i].dax << " " << s_data_from_robot.raw_imu[i].day << " " << s_data_from_robot.raw_imu[i].daz
    //  << " | " <<  s_data_from_robot.raw_imu[i].ddx << " " << s_data_from_robot.raw_imu[i].ddy << " " << s_data_from_robot.raw_imu[i].ddz << "\n";
    
    drc::atlas_raw_imu_t raw;
    raw.utime = s_data_from_robot.raw_imu[i].imu_timestamp;
    raw.packet_count = s_data_from_robot.raw_imu[i].packet_count;
    raw.delta_rotation[0] = s_data_from_robot.raw_imu[i].dax;
    raw.delta_rotation[1] = s_data_from_robot.raw_imu[i].day;
    raw.delta_rotation[2] = s_data_from_robot.raw_imu[i].daz;
    
    raw.linear_acceleration[0] = s_data_from_robot.raw_imu[i].ddx;
    raw.linear_acceleration[1] = s_data_from_robot.raw_imu[i].ddy;
    raw.linear_acceleration[2] = s_data_from_robot.raw_imu[i].ddz;
    imu.raw_imu.push_back( raw );
  }
  lcm_->publish( ("ATLAS_IMU_BATCH") , &imu);
}
*/


void App::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){

}

void App::sendMultisenseState(int64_t utime, float angle){
  pronto::multisense_state_t msg_out;
  msg_out.utime = utime;
  for (std::vector<int>::size_type i = 0; i < 13; i++)  {
    msg_out.joint_name.push_back("z");      
    msg_out.joint_position.push_back(0);      
    msg_out.joint_velocity.push_back(0);
    msg_out.joint_effort.push_back(0);
  }  
  msg_out.num_joints = 13;

  msg_out.joint_position[0] = angle;
  msg_out.joint_name[0] = "hokuyo_joint";

  msg_out.joint_name[1] = "pre_spindle_cal_x_joint";
  msg_out.joint_name[2] = "pre_spindle_cal_y_joint";
  msg_out.joint_name[3] = "pre_spindle_cal_z_joint";

  msg_out.joint_name[4] = "pre_spindle_cal_rol_joint";
  msg_out.joint_name[5] = "pre_spindle_cal_pitch_joint";
  msg_out.joint_name[6] = "pre_spindle_cal_yaw_joint";

  msg_out.joint_name[7] = "post_spindle_cal_x_joint";
  msg_out.joint_name[8] = "post_spindle_cal_x_joint";
  msg_out.joint_name[9] = "post_spindle_cal_x_joint";

  msg_out.joint_name[10] = "post_spindle_cal_roll_joint";
  msg_out.joint_name[11] = "post_spindle_cal_pitch_joint";
  msg_out.joint_name[12] = "post_spindle_cal_yaw_joint";

  lcm_publish_.publish("MULTISENSE_STATE", &msg_out);  
}


int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
/*
arms joint mapping is entered properly

in, in_name, out
0, r_arm_shx, 16
1, r_arm_elx, 17
2, r_leg_akx, 15
3, back_bkx, 2
4, l_arm_wry, 18
5, r_leg_hpy, 12
9, r_arm_wry, 19
10, l_leg_kny, 7
13, l_arm_elx, 20
16, r_leg_aky, 14
20, l_arm_shy, 21
23, r_leg_kny, 13
24, r_arm_wrx, 22
26, l_leg_akx, 9
27, l_arm_ely, 23
28, l_arm_wrx, 24
29, l_leg_hpx, 5
30, l_leg_hpy, 6
31, l_leg_hpz, 4
34, r_leg_hpx, 11
35, l_arm_shx, 25
40, back_bky, 1
42, r_arm_shy, 26
43, neck_ry, 3
45, r_leg_hpz, 10
47, back_bkz, 0
48, l_leg_aky, 8
49, r_arm_ely, 27
*/
  std::vector< std::pair<int,int> > jm;

jm.push_back (  std::make_pair(	0	,	16	));
jm.push_back (  std::make_pair(	1	,	17	));
jm.push_back (  std::make_pair(	2	,	15	));
jm.push_back (  std::make_pair(	3	,	2	));
jm.push_back (  std::make_pair(	4	,	18	));
jm.push_back (  std::make_pair(	5	,	12	));
jm.push_back (  std::make_pair(	9	,	19	));
jm.push_back (  std::make_pair(	10	,	7	));
jm.push_back (  std::make_pair(	13	,	20	));
jm.push_back (  std::make_pair(	16	,	14	));
jm.push_back (  std::make_pair(	20	,	21	));
jm.push_back (  std::make_pair(	23	,	13	));
jm.push_back (  std::make_pair(	24	,	22	));
jm.push_back (  std::make_pair(	26	,	9	));
jm.push_back (  std::make_pair(	27	,	23	));
jm.push_back (  std::make_pair(	28	,	24	));
jm.push_back (  std::make_pair(	29	,	5	));
jm.push_back (  std::make_pair(	30	,	6	));
jm.push_back (  std::make_pair(	31	,	4	));
jm.push_back (  std::make_pair(	34	,	11	));
jm.push_back (  std::make_pair(	35	,	25	));
jm.push_back (  std::make_pair(	40	,	1	));
jm.push_back (  std::make_pair(	42	,	26	));
jm.push_back (  std::make_pair(	43	,	3	));
jm.push_back (  std::make_pair(	45	,	10	));
jm.push_back (  std::make_pair(	47	,	0	));
jm.push_back (  std::make_pair(	48	,	8	));
jm.push_back (  std::make_pair(	49	,	27	));

  int n_joints = jm.size();



  if (js_counter%500 ==0){
    std::cout << "J ST " << js_counter << "\n";
  }  
  js_counter++;
  
  pronto::atlas_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  

  msg_out.joint_position.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_velocity.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.joint_effort.assign(n_joints , std::numeric_limits<int>::min()  );
  msg_out.num_joints = n_joints;

  for (std::vector<int>::size_type i = 0; i < jm.size(); i++)  {
    //std::cout << jm[i].first << " and " << jm[i].second << "\n";
    msg_out.joint_position[ jm[i].second ] = msg->position[ jm[i].first ];      
    msg_out.joint_velocity[ jm[i].second ] = msg->velocity[ jm[i].first ];
    msg_out.joint_effort[ jm[i].second ] = msg->effort[ jm[i].first ];
  }


  for (std::vector<int>::size_type i = 16; i < jm.size(); i++)  {
    msg_out.joint_position[ i ] = 0;      
    msg_out.joint_velocity[ i ] = 0;
    msg_out.joint_effort[ i ] = 0;

  }

  // Append FT sensor info
  pronto::force_torque_t force_torque;
  appendFootSensor(force_torque, foot_sensor_);
  msg_out.force_torque = force_torque;
  lcm_publish_.publish("ATLAS_STATE", &msg_out);
  
  pronto::utime_t utime_msg;
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  utime_msg.utime = joint_utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg); 

  sendMultisenseState(joint_utime, msg->position[22]);
}


void App::appendFootSensor(pronto::force_torque_t& msg_out , trooper_mlc_msgs::FootSensor msg_in){
  msg_out.l_foot_force_z  =  msg_in.left_fz;
  msg_out.l_foot_torque_x =  msg_in.left_mx;
  msg_out.l_foot_torque_y  =  msg_in.left_my;
  msg_out.r_foot_force_z  =  msg_in.right_fz;
  msg_out.r_foot_torque_x  =  msg_in.right_mx;
  msg_out.r_foot_torque_y  =  msg_in.right_my;

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
