# ChassisLib-For-Linux

基于ROS2开发的底盘库

## 1. 环境与依赖

### 1.1 环境

- Ubuntu 22.04

### 1.2 依赖

- ROS 2 Humble
- [MotorLib-For-Linux](https://github.com/BUPT-RobotTeam/MotorLib-For-Linux)

## 2. 使用方法

```shell
cd colcon_ws
cd src
git clone https://github.com/BUPT-RobotTeam/ChassisLib-For-Linux.git
cd ..
colcon build 
```

关于对外接口的使用方法，请参考[ChassisLib-For-Linux-Example](https://github.com/BUPT-RobotTeam/ChassisLib-For-Linux/blob/main/example/chassis_example.cpp)

## 3. TODO

- 正在考虑加入多线程的支持，以提高底盘的控制频率

