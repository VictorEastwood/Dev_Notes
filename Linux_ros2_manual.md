# ROS2 速查手册

## 1.  创建功能包

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

