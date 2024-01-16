#include "chassis_lib/steering_chassis.h"
#include "bupt_can/bupt_can.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

int main()
{
    rclcpp::init(0,nullptr);
    auto can_handle=std::make_shared<Can>("vcan0");
    auto sw_chassis=std::make_shared<SteeringChassis>("/amcl_topic","/cmd_vel",can_handle);
    RCLCPP_INFO(sw_chassis->get_logger(),"Waiting for steering wheel initialization.");
    can_handle->can_start();
    sw_chassis->initialize();
    RCLCPP_INFO(sw_chassis->get_logger(),"Start Control Loop");
    rclcpp::spin(sw_chassis);
    return 0;
}