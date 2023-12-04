#ifndef CHASSIS_H
#define CHASSIS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

class Chassis : public rclcpp::Node
{
protected:
    geometry_msgs::msg::Twist target_vel;
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    double limit_vel,limit_acc;
    double width,length;
    double wheel_radius;
public:
    Chassis(const std::string &pose_topic,const std::string &vel_topic);
    ~Chassis();
    void setVelocity(const geometry_msgs::msg::Twist & cmd_vel);
    void updatePose(const geometry_msgs::msg::PoseStamped & pose);
    virtual void setParameter(const double &limit_vel,const double &limit_acc,const double &wheel_radius,const double &width,const double &length);
    virtual void initialize()=0;
    virtual void execute()=0;
};

#endif // CHASSIS_H