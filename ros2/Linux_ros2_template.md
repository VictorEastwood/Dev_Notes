ROS2 速查模板
---
>本文档主要介绍ROS2的速查模板，包括ROS2的安装、使用、常用命令等相关内容。

<!-- 目录 -->
- [创建功能包](#创建功能包)
- [最简节点模板](#最简节点模板)

# 创建功能包
```bash
ros2 pkg create 功能包名 --build-type 构建类型  --dependencies 依赖
```

ament_cmake

```bash
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
```

ament_python

```bash
ros2 pkg create my_package --build-type ament_python --dependencies rclpy
```

常用依赖

```bash
--dependencies rclcpp geometry_msgs turtlesim tf2 tf2_ros tf2_geometry_msgs
```

# 最简节点模板
MyNode.hpp
```cpp
// file: MyNode.hpp
#ifndef MY_NODE_HPP
#define MY_NODE_HPP

#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode();
private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MY_NODE_HPP
```

MyNode.cpp
```cpp
// file: MyNode.cpp
#include "MyNode.hpp"
MyNode::MyNode() : Node("my_node") {
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MyNode::timer_callback, this));
}

void MyNode::timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Hello, ROS2!");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```
CMakeLists.txt
```cmake

