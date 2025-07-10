# ros2安装
---
## 目录
- [ros2安装](#ros2安装)
  - [目录](#目录)
  - [环境配置](#环境配置)
    - [安装ROS2](#安装ros2)
    - [安装ROS开发必要的工具](#安装ros开发必要的工具)

## 环境配置
###  安装ROS2

输入以下命令安装ros2
```shell
wget http://fishros.com/install -O fishros && . fishros
```
验证是否安装成功
```shell
echo $ROS_DISTRO
```
若返回值为`foxy` `humble` `jazzy`等则安装成功

###  安装ROS开发必要的工具
```shell
sudo apt install -y ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-rqt-* ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-gazebo-* ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers 
```
GAZEBO运行需要在每次打开终端时运行以下命令
```shell 
source /usr/share/gazebo/setup.bash
```

