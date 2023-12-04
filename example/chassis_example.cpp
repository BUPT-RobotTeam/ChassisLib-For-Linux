#include "chassis_lib/omni_chassis.h"
#include "bupt_can/bupt_can.h"
#include <rclcpp/rclcpp.hpp>

int main()
{
    rclcpp::init(0,nullptr);
    auto can_handle = std::make_shared<Can>("can0");
    auto node = std::make_shared<OmniChassis>("/amcl_pose","/cmd_vel",1,can_handle);
    // auto node = std::make_shared<SteeringChassis>("/amcl_pose","/cmd_vel",1,can_handle);
    node->setParameter(1.0,1.0,0.1,0.5,0.5);
    while (rclcpp::ok())
    {
        node->execute();
        rclcpp::spin_some(node);
    }
    return 0;
}