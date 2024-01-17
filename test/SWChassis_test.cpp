#include "chassis_lib/steering_chassis.h"
#include "bupt_can/bupt_can.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

int main()
{
    rclcpp::init(0,nullptr);
    auto can_handle=std::make_shared<Can>("can0");
    auto sw_chassis=std::make_shared<SteeringChassis>("/amcl_topic","/cmd_vel",can_handle);
    RCLCPP_INFO(sw_chassis->get_logger(),"Waiting for steering wheel initialization.");
    sw_chassis->setParameter(5,100,0.053,0.43,0.43);
    can_handle->can_start();
    sw_chassis->initialize();
    RCLCPP_INFO(sw_chassis->get_logger(),"Start Control Loop");
    rclcpp::spin(sw_chassis);
    return 0;
}