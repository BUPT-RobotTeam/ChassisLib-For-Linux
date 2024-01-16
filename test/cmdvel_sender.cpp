#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

int main()
{
    rclcpp::init(0,nullptr);
    rclcpp::Node cmdvel_sender("cmdvel_sender");
    auto cmdvel_pub=cmdvel_sender.create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    while (rclcpp::ok())
    {
        std::cout<<"Input linear velocity and angular velocity as [x y w]:"<<std::endl;
        double linear_x,linear_y,angular_z;
        std::cin>>linear_x>>linear_y>>angular_z;
        geometry_msgs::msg::Twist msg;
        msg.linear.x=linear_x;
        msg.linear.y=linear_y;
        msg.angular.z=angular_z;
        cmdvel_pub->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}