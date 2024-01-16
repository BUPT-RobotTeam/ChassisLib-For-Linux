#include "chassis_lib/base_chassis.h"
#include <string>

Chassis::Chassis(const std::string &pose_topic,const std::string &vel_topic) : Node("chassis_node")
{
    vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(vel_topic, 10, std::bind(&Chassis::setVelocity,this,std::placeholders::_1));
    pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&Chassis::updatePose,this,std::placeholders::_1));
    //create a timer to execute the control loop
    timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Chassis::execute,this));
}

void Chassis::setVelocity(const geometry_msgs::msg::Twist & cmd_vel) {
    target_vel = cmd_vel;
    RCLCPP_INFO(this->get_logger(),"Receive Twist Msg %.2f %.2f %.2f",target_vel.linear.x,target_vel.linear.y,target_vel.angular.z);
}

void Chassis::updatePose(const geometry_msgs::msg::PoseStamped & pose) {
    current_pose = pose;
}

void Chassis::setParameter(const double &limit_vel, const double &limit_acc, 
                           const double &width, const double &length, 
                           const double &wheel_radius, const double &ratio)
{
    this->limit_vel=limit_vel;
    this->limit_acc=limit_acc;
    this->width=width;
    this->length=length;
    this->wheel_radius=wheel_radius;
    this->ratio=ratio;
}
