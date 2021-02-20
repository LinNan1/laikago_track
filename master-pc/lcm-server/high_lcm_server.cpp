/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "laikago_sdk/laikago_sdk.hpp"

#include <math.h>

using namespace laikago;

HighCmd cmd = {0};
HighState state = {0};
Control control(HIGHLEVEL);
// 接收状态的端口：8008
// 向192.168.123.10:8007 发送指令
// wifi传输
UDP udp(HIGH_CMD_LENGTH, HIGH_STATE_LENGTH);
LCM mylcm;

void UDPRecv()
{
    // 时刻接收udp包保证实时,  from laikago
    udp.Recv();
}

void LCMRecv()
{
    // 时刻轮询订阅的话题
    // Subscribe channel: LCM_High_Cmd
    mylcm.Recv();
}

void RobotControl() 
{
    udp.GetState(state);
    // udp包数据->lcm
    mylcm.Send(state); // Publish channel: LCM_High_State

    // printf("\e[1A\r");
    // printf("forwardSpeed: %f\n", state.forwardSpeed);
    // printf("bodyHeight  : %f", state.bodyHeight);
    // fflush(stdout);
    mylcm.Get(cmd);

    // lcm获取的命令->udp发出去

    printf("\rforwardSpeed: %f", cmd.forwardSpeed);
    fflush(stdout);
    udp.Send(cmd);
}

int main(void) 
{
    control.loop.SetLCM(true);
    control.loop.SetLCMPeriod(4000); //4ms
    mylcm.SubscribeCmd();
    control.InitCmdData(cmd);
    control.loop.RegistFunc("UDP/Send", RobotControl);
    control.loop.RegistFunc("UDP/Recv", UDPRecv);
    control.loop.RegistFunc("LCM/Recv", LCMRecv);
    control.loop.Start();
    return 0; 
}
