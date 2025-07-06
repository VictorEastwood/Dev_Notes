# ros2初步

--- 

## 1.环境配置
### 1.1 安装ROS2

输入以下命令安装ros2
```shell
wget http://fishros.com/install -O fishros && . fishros
```
验证是否安装成功
```shell
echo $ROS_DISTRO
```
若返回值为`foxy` `humble` `jazzy`等则安装成功

### 1.2 安装ROS开发必要的工具
```shell
sudo apt install -y ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-rqt-* ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-gazebo-* ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers 
```
GAZEBO运行需要在每次打开终端时运行以下命令
```shell 
source /usr/share/gazebo/setup.bash
```

### 1.3 安装KDL库
首先，确保你已经安装了必要的依赖库：
```
sudo apt-get update
sudo apt-get install ros-humble-kdl-parser
```
克隆KDL库
```
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
```
编译安装
```bash
cd orocos_kinematics_dynamics
cd orocos_kdl
mkdir build
cd build
```
使用cmake-curses-gui配置编译选项
```bash
ccmake ..
```

按下c键 你会看到如下界面通过上下键选择你需要的选项，通过enter键修改BUILD_MODELS和ENABLE_EXAMPLES为ON，然后按下c键保存退出

编译
```bash
make
```
安装
```bash
sudo make install
```
### 1.4 安装moveit2
官方文档链接：[moveit2](https://moveit.picknik.ai/humble/index.html)
安装rosdep来安装系统依赖项：
```bash
sudo apt install python3-rosdep
```
初始化rosdep
```bash
sudo rosdep init
```
如果初始化rosdep出现了错误：可以尝试以下方法：
进入/etc文件夹
```bash
cd /etc
```
修改hosts文件
```bash
sudo gedit hosts
```
在末尾添加以下内容
```bash
151.101.84.133 raw.githubusercontent.com
```
更新rosdep
```bash
rosdep update
```
```bash
sudo apt update
sudo apt dist-upgrade
```
使用mixin安装Colcon ROS 2 构建系统：
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```
安装vcstool：
```bash
sudo apt install python3-vcstool
```
设置colcon工作区:
```bash
mkdir -p ~/ws_moveit2/src
```
进入 Colcon 工作区并拉取 MoveIt 教程源：
```bash
cd ~/ws_moveit2/src
git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
```
下载 MoveIt 其余部分的源代码：
```bash
vcs import < moveit2_tutorials/moveit2_tutorials.repos
```
安装 MoveIt 及其所有依赖项：
```bash
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```
编译 MoveIt：
```bash
cd ~/ws_moveit2
colcon build --mixin release --parallel-workers 1
```
获取 MoveIt 2 的环境变量：
```bash
source ~/ws_moveit2/install/setup.bash
```
添加环境变量到 ~/.bashrc 文件：
```bash
echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc
```
切换到 Cyclone DDS 作为 ROS 2 的默认通信中间件：
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
# You may want to add this to ~/.bashrc to source it automatically
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
## 3.ROS2常用命令

### 3.1 ros2 节点相关
查看所有节点
```shell
ros2 node list
```
查看节点信息
```shell
ros2 node info /node_name
```
### 3.2 ros2 话题相关
查看所有话题
```shell
ros2 topic list
```
查看话题信息
```shell
ros2 topic info /topic_name
```
### 3.3 ros2 服务相关
查看所有服务
```shell
ros2 service list
```
查看服务信息
```shell
ros2 service info /service_name
```
### 3.4 ros2 参数相关
查看所有参数
```shell
ros2 param list
```
查看参数信息
```shell
ros2 param get /param_name
```
### 3.5 运行ros2程序
运行节点
```shell
ros2 run package_name executable_name
```
运行launch文件
```shell
ros2 launch package_name launch_file_name
```
---
## 4.ROS2常用工具
### ROS2 Control
以下是 **ROS 2 control** 每个命令的示例代码块，供你参考使用：  

 1. `list_controller_types`
    列出可用控制器类型及其基类：  
    ```bash
    ros2 control list_controller_types
    ```
 2. `list_controllers`
    列出当前加载的控制器及其状态：  
    ```bash
    ros2 control list_controllers
    ```
 3. `list_hardware_components`
    列出可用硬件组件：  
    ```bash
    ros2 control list_hardware_components
    ```
 4. `list_hardware_interfaces`
    列出可用的指令接口和状态接口：  
    ```bash
    ros2 control list_hardware_interfaces
    ```
5. `load_controller`
    加载指定的控制器：  
    ```bash
    ros2 control load_controller <controller_name>
    ```
    **示例**：加载名为 `joint_trajectory_controller` 的控制器：  
    ```bash
    ros2 control load_controller joint_trajectory_controller
    ```
6. `reload_controller_libraries`
    重新加载控制器库：  
    ```bash
    ros2 control reload_controller_libraries
    ```
7. `set_controller_state`
    设置控制器的状态（如 `active` 或 `inactive`）：  
    ```bash
    ros2 control set_controller_state <controller_name> <state>
    ```
    **示例**：将控制器 `joint_trajectory_controller` 设置为 `active` 状态：  
    ```bash
    ros2 control set_controller_state joint_trajectory_controller active
    ```
8. `set_hardware_component_state`
    设置硬件组件的状态：  
    ```bash
    ros2 control set_hardware_component_state <hardware_component_name> <state>
    ```
    **示例**：将硬件组件 `robot_arm` 设置为 `active` 状态：  
    ```bash
    ros2 control set_hardware_component_state robot_arm active
    ```
9. `switch_controllers`
    启动和停止指定控制器，实现控制器切换：  
    ```bash
    ros2 control switch_controllers --start-controllers <controller_name> --stop-controllers <controller_name> [--strict] [--timeout <seconds>]
    ```
    **示例**：启动 `joint_trajectory_controller`，停止 `position_controller`：  
    ```bash
    ros2 control switch_controllers --start-controllers joint_trajectory_controller --stop-controllers position_controller
    ```
10. `unload_controller`
    卸载指定控制器：  
    ```bash
    ros2 control unload_controller <controller_name>
    ```
    **示例**：卸载名为 `joint_trajectory_controller` 的控制器：  
    ```bash
    ros2 control unload_controller joint_trajectory_controller
    ```

11. `view_controller_chains`
    生成链式控制器图表并保存为 PDF 文件：  
    ```bash
    ros2 control view_controller_chains
    ```
    **输出**：图表会保存到 `/tmp/controller_diagram.gv.pdf`。

    每个命令均可通过追加 `-h` 获取详细帮助，例如：  
    ```bash
    ros2 control switch_controllers -h
    ```

### moveit2
官方文档链接：[moveit2](https://moveit.picknik.ai/humble/index.html)
启动moveit2 assistant
```shell
ros2 run moveit_setup_assistant moveit_setup_assistant
```

