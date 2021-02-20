#include "libs/laikago_comm.h"
#include "libs/laikago_pycomm.h"
#include "libs/Robot.h"
#include <math.h>

#ifdef ROBOTSERVER

static long motiontime = 0;
static float oldyaw = 0;
const char* to_addr = "127.0.0.1";
int to_port = 8002;

void Setup()
{
    SetLevel(HIGHLEVEL);
    return;
}

void RobotControl(void) 
{
    motiontime = motiontime+2; // 这里motiontime的1000约为1s
    // RobotStatus();

    SendHigh.forwardSpeed = 0.0f;
    SendHigh.sideSpeed = 0.0f;
    SendHigh.rotateSpeed = 0.0f;
    SendHigh.forwardSpeed = 0.0f;

    SendHigh.mode = 0;
    SendHigh.roll  = 0;
    SendHigh.pitch = 0;
    SendHigh.yaw = 0;
    
    // if(!getCmd(&SendHigh)){
    //     printf("SendHigh.forwardSpeed: %f\n",SendHigh.forwardSpeed);
    //     printf("SendHigh.sideSpeed   : %f\n",SendHigh.sideSpeed);
    //     printf("SendHigh.rotateSpeed : %f\n",SendHigh.rotateSpeed);
    //     printf("SendHigh.mode        : %d\n",SendHigh.mode);
    //     printf("SendHigh.roll        : %f\n",SendHigh.roll);
    //     printf("SendHigh.pitch       : %f\n",SendHigh.pitch);
    //     printf("SendHigh.yaw         : %f\n",SendHigh.yaw);
    //     printf("-----------------------------\n");
    // }
    
    sendStat(RecvHigh, to_addr, to_port);

    // if(motiontime>1000 && motiontime<1500){
    //     SendHigh.roll = 0.5f; // 顺时针
    // }

    // if(motiontime>1500 && motiontime<2000){
    //     SendHigh.pitch = 0.3f; // 低头
    // }

    // if(motiontime>2000 && motiontime<2500){
    //     SendHigh.yaw = 0.3f; //偏航
    // }

    // if(motiontime>2500 && motiontime<3000){
    //     SendHigh.bodyHeight = -0.3f;
    // }

    // if(motiontime>3000 && motiontime<3500){
    //     SendHigh.bodyHeight = 0.3f;
    // }

    // if(motiontime>3500 && motiontime<4000){
    //     SendHigh.bodyHeight = 0.0f;
    // }

    // if(motiontime>4000 && motiontime<5000){
    //     SendHigh.mode = 2; // 行走
    // }

    // if(motiontime>5000 && motiontime<8500){
    //     SendHigh.forwardSpeed = 0.2f; // -1  ~ +1
    // }

    // if(motiontime>8500 && motiontime<12000){
    //     SendHigh.forwardSpeed = -0.2f; // -1  ~ +1
    // }

    // if(motiontime>12000 && motiontime<16000){
    //     oldyaw = RecvHigh.imu.rpy[2];
    //     //if(fabs(RecvHigh.imu.rpy[2] - oldyaw) < 60) 
    //     SendHigh.rotateSpeed = 0.5f;   // turn
    //     //std::cout<<RecvHigh.imu.rpy[2] - oldyaw<<std::endl;
    // }

    // if(motiontime>16000 && motiontime<20000){
    //     oldyaw = RecvHigh.imu.rpy[2];
    //     if(fabs(RecvHigh.imu.rpy[2] - oldyaw) < 60) SendHigh.rotateSpeed = -0.5f;   // turn
    // }

    // if(motiontime>20000 && motiontime<21000){
    //     SendHigh.mode = 1;
    // }
}

int main(void) 
{
    pycommInit();
    Setup();
    ControlStart(); //start send and recv threads
    
    return 0; 
}

#else
#endif
