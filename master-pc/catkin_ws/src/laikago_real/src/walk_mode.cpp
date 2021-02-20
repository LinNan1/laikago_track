/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <laikago_msgs/HighCmd.h>
#include <laikago_msgs/HighState.h>
#include "laikago_sdk/laikago_sdk.hpp"

using namespace laikago;

static long motiontime = 0;
HighCmd SendHighLCM = {0};
HighState RecvHighLCM = {0};
laikago_msgs::HighCmd SendHighROS;


Control control(HIGHLEVEL);
LCM roslcm;
boost::mutex mutex;

void* update_loop(void* data)
{
    while(ros::ok){
        boost::mutex::scoped_lock lock(mutex);
        roslcm.Recv();
        lock.unlock();
        usleep(2000);
    }
}

void commandCallback(const laikago_msgs::HighCmd &cmd_msg){
    memcpy(&SendHighLCM, &cmd_msg, sizeof(HighCmd));
    // printf("debug: %f\n",SendHighLCM.forwardSpeed); OK
    roslcm.Send(SendHighLCM);
}

int main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl;
     //         << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    ros::init(argc, argv, "walk_ros_mode");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    roslcm.SubscribeState();
    ros::Publisher high_state_pub = n.advertise<laikago_msgs::HighState>("/laikago_real/high_state", 1000);
    ros::Subscriber high_cmd_sub = n.subscribe("/laikago_real/high_cmd", 1000, commandCallback);
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, NULL);

    while (ros::ok()){
        roslcm.Get(RecvHighLCM);
        laikago_msgs::HighState RecvHighROS;
        
        memcpy(&RecvHighROS, &RecvHighLCM, sizeof(HighState));
        
        high_state_pub.publish(RecvHighROS);

        roslcm.Send(SendHighLCM); // 不安全, debug使用
        
        // printf("%f\n",  RecvHighROS.forwardSpeed);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

