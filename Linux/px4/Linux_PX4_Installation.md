# PX4 Linux 安装指南

## 环境要求

- Ubuntu 22.04 LTS
- ROS 2 Humble

本文默认已安装 ROS 2 Humble。若尚未安装，请先参考 [ROS 2 安装与配置](../ros2/Linux_ros2_installation.md)。

## 1. 安装 PX4

### 1.1 克隆代码仓库

递归克隆 PX4 仓库及其子模块：

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

### 1.2 配置开发环境

运行仓库提供的 Ubuntu 环境配置脚本：

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

### 1.3 编译 SITL

进入 PX4 仓库，编译软件在环仿真（SITL）目标：

```bash
cd PX4-Autopilot/
make px4_sitl
```

### 1.4 安装 Python 依赖

```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

## 2. 安装 Micro XRCE-DDS Agent

以下命令使用 `v2.4.3` 版本。

### 2.1 克隆代码仓库

在新的终端中，进入希望存放源码的目录，再执行：

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
```

### 2.2 配置并编译

```bash
mkdir build
cd build
cmake ..
make
```

### 2.3 安装并更新动态链接库缓存

```bash
sudo make install
sudo ldconfig /usr/local/lib/
```
