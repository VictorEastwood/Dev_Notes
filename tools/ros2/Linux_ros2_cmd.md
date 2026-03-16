# ROS2常用命令

> 本文档主要介绍ROS2的常用命令和使用方法，包括节点、话题、服务、参数等相关操作。

## 目录
- [ROS2常用命令](#ros2常用命令)
  - [目录](#目录)
  - [ROS2消息相关](#ros2消息相关)
    - [ros2 节点](#ros2-节点)
    - [ros2 话题](#ros2-话题)
    - [ros2 服务](#ros2-服务)
    - [ros2 参数](#ros2-参数)
  - [运行ros2程序](#运行ros2程序)
## ROS2消息相关
### ros2 节点
查看所有节点
```shell
ros2 node list
```
查看节点信息
```shell
ros2 node info /node_name
```

### ros2 话题
查看所有话题
```shell
ros2 topic list
```
查看话题信息
```shell
ros2 topic info /topic_name
```

### ros2 服务
查看所有服务
```shell
ros2 service list
```
查看服务信息
```shell
ros2 service info /service_name
```

### ros2 参数
查看所有参数
```shell
ros2 param list
```
查看参数信息
```shell
ros2 param get /param_name
```

## 运行ros2程序
运行节点
```shell
ros2 run package_name executable_name
```
运行launch文件
```shell
ros2 launch package_name launch_file_name
```



