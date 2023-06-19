# UR5 Pick And Place demo
This repository is a ROS 2 package that contains Pick And Place demo for UR5 robot.

## Prerequisites
 * [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git)
```bash
sudo apt-get install ros-foxy-ur-robot-driver
```

## Launching the demo
```bash
ros2 launch ur5_example demo.launch.py [ur_type:=ur5] [robot_ip:=yyy.yyy.yyy.yyy] [use_fake_hardware:=true]
```