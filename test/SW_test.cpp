/**
 * @brief 舵轮测试程序
 * @author woshirem
 * @date 2024.1.15
 **/
#include "bupt_can/bupt_can.h"
#include "chassis_lib/steering_wheel.h"
#include <condition_variable>
#include <iostream>
#include <chrono>
#include <thread>

std::array<int16_t,2> angle={0,0};
std::array<int16_t,2> vel={0,0};

int main()
{
    auto can_handle = std::make_shared<Can>("vcan0");
    auto steering_wheel = std::make_shared<SteeringWheel>(0x300,can_handle);
    can_handle->can_start();

    // 舵轮控制
    while (true)
    {
        int16_t pos=0,rpm=0;
        scanf("%hd %hd",&pos,&rpm);
        if (pos==-1&&rpm==-1)
        {
            break;
        }
        angle[0]=pos*10,vel[0]=rpm*6;
        steering_wheel->sendCommand(angle,vel);
    }
    return 0;
}