#include "omni_chassis.h"
#include "bupt_can/bupt_can.h"
#include <rclcpp/rclcpp.hpp>

int main()
{
    rclcpp::init(0,nullptr);
    auto node = rclcpp::Node::make_shared("omni_chassis_example");

    auto can_handle = std::make_shared<Can>("can0");
    auto chassis = std::make_shared<OmniChassis>(1,can_handle);

    auto vel_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&Chassis::setVelocity,chassis,std::placeholders::_1));
    rclcpp::Rate(1).sleep();
    chassis->initialize();
    RCLCPP_INFO(node->get_logger(),"Chassis initialized.");
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        chassis->execute();
        rate.sleep();
        rclcpp::spin_some(node);
    }
    return 0;
}