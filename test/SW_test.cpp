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
    auto can_handle = std::make_shared<Can>("can0");
    auto steering_wheel = std::make_shared<SteeringWheel>(0x301,can_handle);
    can_handle->can_start();

    while (true)
    {
        std::cout<<"Please input the angle of the steering wheel (0~360):"<<std::endl;
        std::cin>>angle[1];
        std::cout<<"Please input the velocity of the steering wheel (0~100):"<<std::endl;
        std::cin>>vel[1];
        angle[1]*=10;
        vel[1]*=6;
        steering_wheel->sendCommand(angle,vel);
    }
    return 0;
}