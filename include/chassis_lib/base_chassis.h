#ifndef CHASSIS_H
#define CHASSIS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Chassis
{
protected:
    geometry_msgs::msg::Twist target_vel;
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Node::SharedPtr node;
public:
    Chassis();
    ~Chassis();
    void setVelocity(const geometry_msgs::msg::Twist::SharedPtr & cmd_vel);
    void updatePose(const geometry_msgs::msg::PoseStamped::SharedPtr & pose);
    virtual void initialize()=0;
    virtual void execute()=0;
};

#endif // CHASSIS_H