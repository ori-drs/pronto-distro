#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/microstrain_comm.hpp"
#include "lcmtypes/bot_core.hpp"

class Handler 
{
    public:
        Handler(lcm::LCM* lcm_, std::string chan_out_):lcm_(lcm_), chan_out_(chan_out_){};

        ~Handler() {}
        lcm::LCM* lcm_;
        std::string chan_out_;

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const microstrain::ins_t* msg)
        {
            bot_core::pose_t pose;
            pose.utime = msg->utime;
            double zeros[]={0,0,0};

            memcpy(pose.pos,zeros, sizeof(double)*3);
            memcpy(pose.vel,zeros, sizeof(double)*3);
            memcpy(pose.rotation_rate,zeros, sizeof(double)*3);
            memcpy(pose.accel,zeros, sizeof(double)*3);

            pose.orientation[0] = msg->quat[0];
            pose.orientation[1] = msg->quat[1];
            pose.orientation[2] = msg->quat[2];
            pose.orientation[3] = msg->quat[3];

            lcm_->publish(chan_out_.c_str() , &pose);
        }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    std::string chan_out = "POSE";
    if (argc > 1){
      chan_out = argv[1];
    }
    printf("Will output quaternion from MICROSTRAIN_INS as part of %s\n",chan_out.c_str() );
    

    if(!lcm.good())
        return 1;

    Handler handlerObject(&lcm, chan_out);
    lcm.subscribe("MICROSTRAIN_INS", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
