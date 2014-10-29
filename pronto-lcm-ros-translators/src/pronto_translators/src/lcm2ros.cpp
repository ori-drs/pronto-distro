#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/pose_t.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <map>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;

    void poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
    ros::Publisher pose_body_pub_;
    ros::NodeHandle* rosnode;    
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  lcm_->subscribe("POSE_BODY",&LCM2ROS::poseBodyHandler, this);
  pose_body_pub_ = nh_.advertise<nav_msgs::Odometry>("/pose_body",10);
  rosnode = new ros::NodeHandle();
}

void LCM2ROS::poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg) {
  //ROS_ERROR("LCM2ROS got pose_t");
  nav_msgs::Odometry msgout;
  msgout.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  msgout.pose.pose.position.x = msg->pos[0];
  msgout.pose.pose.position.y = msg->pos[1];
  msgout.pose.pose.position.z = msg->pos[2];
  msgout.pose.pose.orientation.w = msg->orientation[0];
  msgout.pose.pose.orientation.x = msg->orientation[1];
  msgout.pose.pose.orientation.y = msg->orientation[2];
  msgout.pose.pose.orientation.z = msg->orientation[3];
  msgout.twist.twist.linear.x = msg->vel[0];
  msgout.twist.twist.linear.y = msg->vel[1];
  msgout.twist.twist.linear.z = msg->vel[2];
  msgout.twist.twist.angular.x = msg->rotation_rate[0];
  msgout.twist.twist.angular.y = msg->rotation_rate[1];
  msgout.twist.twist.angular.z = msg->rotation_rate[2];
  pose_body_pub_.publish(msgout);
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"lcm2ros",ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  LCM2ROS handlerObject(lcm, nh);
  cout << "\nlcm2ros translator ready\n";
  ROS_ERROR("LCM2ROS Translator Ready");
  
  while(0 == lcm->handle());
  return 0;
}
