# PX4与ROS2的结合开发
---
### PX4 Linux安装
环境:
- Ubuntu 22.04 LTS
- ROS2 Humble
### 1. 安装PX4
克隆PX4代码库并编译
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
安装ROS2以及依赖
默认已经安装了ROS2 Humble，如果没有安装，请参考[ROS2 安装与配置](/ros2/Linux_ros2_installation.md)
以下是PX4所需的依赖安装命令：
```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

### 2. 安装Micro XRCE-DDS Agent
```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
```
你可能会遇到以下报错：
```bash
fatal: invalid reference: 2.12.x
CMake Error at /home/ubuntu/Micro-XRCE-DDS-Agent/build/fastdds/tmp/fastdds-gitclone.cmake:40 (message):
  Failed to checkout tag: '2.12.x'


make[2]: *** [CMakeFiles/fastdds.dir/build.make:101: fastdds/src/fastdds-stamp/fastdds-download] Error 1
make[1]: *** [CMakeFiles/Makefile2:141: CMakeFiles/fastdds.dir/all] Error 2
make: *** [Makefile:91: all] Error 2
```
这个错误是因为 FastDDS 的 git 标签 2.12.x checkout 失败导致的。我们可以查看可用的 FastDDS 版本：
```bash
curl -s https://api.github.com/repos/eProsima/Fast-DDS/releases | grep '"tag_name"' | head -10
```
输出结果可能类似于：
```bash
    "tag_name": "v3.3.0",
    "tag_name": "v3.1.3",
    "tag_name": "v2.10.7",
    "tag_name": "v3.2.2",
    "tag_name": "v2.6.10",
    "tag_name": "v3.2.1",
    "tag_name": "v3.2.0",
    "tag_name": "v3.0.2",
    "tag_name": "v3.1.2",
    "tag_name": "v3.1.1",
```
我们可以选择一个可用的版本，建议使用v2.10.7版本。修改CMakeLists.txt文件中的FastDDS版本： 在CMakeLists.txt中找到98,99行
```txt
    set(_fastdds_version 2.12)
    set(_fastdds_tag 2.12.x)
```
将该两行修改为：
```txt
    set(_fastdds_version 2.10)
    set(_fastdds_tag v2.10.7)
```
重复步骤3，重新编译Micro XRCE-DDS Agent成功即可进行安装：
```bash
sudo make install
sudo ldconfig /usr/local/lib/
```
使用连接到模拟器上运行的 uXRCE-DDS 客户端的设置启动代理：
```bash
MicroXRCEAgent udp4 -p 8888
```
 在另一个终端中，运行 PX4 SITL 模拟器：
```bash
make px4_sitl gz_x500
```
效果:
PX4 终端在 NuttShell/PX4 系统控制台启动和运行时显示输出。一旦代理连接，输出应该包括 INFO 消息，显示数据写入器的创建：
```bash
...
INFO  [uxrce_dds_client] synchronized with time offset 1675929429203524us
INFO  [uxrce_dds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 83
INFO  [uxrce_dds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 168
INFO  [uxrce_dds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 188
...
```
micro XRCE-DDS Agent 终端显示代理已连接到 PX4 SITL 模拟器：
```bash
...
[1675929445.268957] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x00000001, publisher_id: 0x0DA(3), participant_id: 0x001(1)
[1675929445.269521] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x00000001, datawriter_id: 0x0DA(5), publisher_id: 0x0DA(3)
[1675929445.270412] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x0DF(2), participant_id: 0x001(1)
...
```
### 3.构建 ROS 2 工作空间
拉取示例代码
```bash
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```
编译 ROS 2 工作空间
```bash
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

如果出现问题：
```bash
pip install setuptools==65.5.1
```







