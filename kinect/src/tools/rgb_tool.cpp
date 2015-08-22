// input: KINECT_FRAME
// output: KINECT_RGB

#include <stdio.h>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/kinect.hpp"
class rgb_tool{
  public:
    rgb_tool(lcm::LCM* &lcm_);
    ~rgb_tool(){}
    
  private:
    lcm::LCM* lcm_;
    
    void kinectHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  kinect::frame_msg_t* msg); 
    
    int decimate_;
};    

rgb_tool::rgb_tool(lcm::LCM* &lcm_): lcm_(lcm_){
        
  lcm_->subscribe( "KINECT_FRAME",&rgb_tool::kinectHandler,this);

}

void rgb_tool::kinectHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  kinect::frame_msg_t* msg){


  bot_core::image_t lcm_img;
  lcm_img.utime =msg->image.timestamp;
  lcm_img.width =msg->image.width;
  lcm_img.height =msg->image.height;
  lcm_img.nmetadata =0;
  int n_colors = 3;
  lcm_img.row_stride=n_colors*msg->image.width;

  if (msg->image.image_data_format == kinect::image_msg_t::VIDEO_RGB){
    lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    lcm_img.size = msg->image.image_data_nbytes;
    lcm_img.data = msg->image.image_data;
  }else if (msg->image.image_data_format == kinect::image_msg_t::VIDEO_RGB_JPEG){
    lcm_img.data = msg->image.image_data;
    lcm_img.size = msg->image.image_data_nbytes;
    lcm_img.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
  }else{
    std::cout << "Format not recognized: " << msg->image.image_data_format << std::endl;
  }

  lcm_->publish("KINECT_RGB", &lcm_img);

}

int main(int argc, char ** argv) {

  lcm::LCM* lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  rgb_tool app(lcm);
  std::cout << "Ready image tool" << std::endl << "============================" << std::endl;
  while(0 == lcm->handle());
  return 0;
}
