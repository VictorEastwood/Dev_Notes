# ROS2 速查模板

> 本文档主要介绍ROS2的速查模板，包括ROS2的安装、使用、常用命令等相关内容。

<!-- 目录 -->
- [创建功能包](#创建功能包)
- [创建消息接口](#创建消息接口)
- [最简节点模板](#最简节点模板)

## 创建功能包

```bash
ros2 pkg create 功能包名 --build-type 构建类型 --dependencies 依赖
```

**ament_cmake**
```bash
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
```

**ament_python**
```bash
ros2 pkg create my_package --build-type ament_python --dependencies rclpy
```

**常用依赖**
```bash
--dependencies rclcpp geometry_msgs turtlesim tf2 tf2_ros tf2_geometry_msgs
```

## 创建消息接口

### 1. 创建自定义消息功能包

```bash
# 创建专门的消息功能包
ros2 pkg create my_interfaces --build-type ament_cmake --dependencies rosidl_default_generators builtin_interfaces geometry_msgs std_msgs
```

### 2. 消息文件目录结构

```
my_interfaces/
├── CMakeLists.txt
├── msg/
│   ├── CustomMessage.msg
│   └── VehicleState.msg
├── srv/
│   └── Calculate.srv
└── package.xml
```

### 3. 消息文件示例

**msg/CustomMessage.msg**
```
# 自定义消息示例
std_msgs/Header header
string message_content
int32 data_value
float64[3] position
geometry_msgs/Point point
```

**可用数据类型：**
- 基本类型：`bool, byte, char, float32, float64, int8, uint8, int16, uint16, int32, uint32, int64, uint64, string, wstring`
- 数组：`int32[]`（动态数组）, `int32[3]`（固定数组）, `int32[<=3]`（有界数组）
- 嵌套类型：`std_msgs/Header`, `geometry_msgs/Point` 等

**msg/VehicleState.msg**
```
# 车辆状态消息
std_msgs/Header header
float64 velocity
float64 steering_angle
float64 acceleration
bool emergency_stop
int32[3] rgb_led
string[<=5] warnings  # 最多5个警告信息
```

**srv/Calculate.srv**
```
# 请求部分
float64 a
float64 b
string operation
---
# 响应部分
float64 result
bool success
string message
```

### 4. 配置CMakeLists.txt

在`my_interfaces/CMakeLists.txt`中添加：

```cmake
# 查找依赖包
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "msg/VehicleState.msg"
  "srv/Calculate.srv"
  DEPENDENCIES geometry_msgs std_msgs
)
```

### 5. 配置package.xml

在`my_interfaces/package.xml`中添加：

```xml
<depend>builtin_interfaces</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```



### 7. 在其他包中使用自定义消息

在`CMakeLists.txt`中添加依赖：

```cmake
find_package(my_interfaces REQUIRED)

# 在可执行目标中添加依赖
ament_target_dependencies(your_node
  rclcpp
  my_interfaces
  # 其他依赖...
)
```

在`package.xml`中添加：

```xml
<depend>my_interfaces</depend>
```

## 创建最简节点模板

### C++ 最简节点模板

在CMakeLists.txt中添加：

```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_node_1 src/cpp_node_1.cpp)
ament_target_dependencies(cpp_node_1 rclcpp)

add_executable(cpp_node_2 src/cpp_node_2.cpp)
ament_target_dependencies(cpp_node_2 rclcpp)
install(TARGETS
  cpp_node_1
  cpp_node_2
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
```

最简化的C++节点代码示例：

```cpp
#include "rclcpp/rclcpp.hpp"

class CustomNode : public rclcpp::Node
{
public:
  CustomNode() : Node("custom_node")
  {
    RCLCPP_INFO(this->get_logger(), "Hello from CustomNode");
  }
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## 创建定时器节点模板

### C++ 定时器节点模板

```cpp
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class CustomNode : public rclcpp::Node
{
public:
    CustomNode();  
    
private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
};


CustomNode::CustomNode() : Node("custom_node")
{
    RCLCPP_INFO(this->get_logger(), "Hello from CustomNode");
    
    // 创建定时器，1000ms周期
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CustomNode::timer_callback, this)
    );
}


void CustomNode::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 消息接口

### 话题发布与订阅

假定有一个自定义消息`CustomMsg.msg`，内容如下：

```
# CustomMsg.msg
std_msgs/Header header
int32 data
float64 timestamp
int32[] data_array
string message
```

话题通信示例：
```cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/custom_msg.hpp"
#include <chrono>
#include <vector>

class CustomNode : public rclcpp::Node
{
public:
    CustomNode();
    
private:
    void timer_callback();
    void subscription_callback(const custom_msgs::msg::CustomMsg::SharedPtr msg);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::CustomMsg>::SharedPtr publisher_;
    rclcpp::Subscription<custom_msgs::msg::CustomMsg>::SharedPtr subscription_;
    int counter_ = 0;
};

// 类外定义构造函数
CustomNode::CustomNode() : Node("custom_node")
{
    RCLCPP_INFO(this->get_logger(), "CustomNode started");
    
    // 创建发布者
    publisher_ = this->create_publisher<custom_msgs::msg::CustomMsg>(
        "custom_topic", 10);
    
    // 创建订阅者
    subscription_ = this->create_subscription<custom_msgs::msg::CustomMsg>(
        "custom_topic", 10,
        std::bind(&CustomNode::subscription_callback, this, std::placeholders::_1));
    
    // 创建定时器 (1000ms周期)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CustomNode::timer_callback, this));
}

// 类外定义定时器回调函数 - 发布消息
void CustomNode::timer_callback()
{
    auto message = custom_msgs::msg::CustomMsg();
    
    // 填充消息内容
    message.header.stamp = this->now();
    message.header.frame_id = "custom_frame";
    message.data = counter_;
    message.timestamp = this->now().seconds();
    message.message = "Hello from CustomNode";
    
    // 填充数组数据
    message.data_array = {counter_, counter_ + 1, counter_ + 2};
    
    // 发布消息
    publisher_->publish(message);
    
    RCLCPP_INFO(this->get_logger(), 
                "Published: data=%d, array_size=%zu", 
                message.data, message.data_array.size());
    
    counter_++;
}

// 类外定义订阅回调函数 - 接收消息
void CustomNode::subscription_callback(const custom_msgs::msg::CustomMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), 
                "Received: data=%d, timestamp=%.2f, message='%s', array=[%d, %d, %d]",
                msg->data,
                msg->timestamp,
                msg->message.c_str(),
                msg->data_array[0],
                msg->data_array[1],
                msg->data_array[2]);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```







