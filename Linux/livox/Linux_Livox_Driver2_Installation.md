# Livox Driver 2 安装准备

本文整理 Livox-SDK2 及 PCL 相关依赖的安装步骤。

## 1. 安装 Livox-SDK2

Livox-SDK2 是用于与 Livox 激光雷达通信的软件开发工具包。

### 1.1 依赖要求

- [CMake](https://cmake.org/) 3.0.0 或更高版本
- GCC 4.8.1 或更高版本

安装 CMake：

```bash
sudo apt install cmake
```

### 1.2 克隆代码仓库

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
```

### 1.3 配置并编译

创建构建目录并生成构建文件：

```bash
mkdir build
cd build
cmake ..
```

编译成功后再执行安装步骤：

```bash
make -j
```

### 1.4 安装 SDK

```bash
sudo make install
```

安装后的文件位置：

| 文件类型 | 安装目录 |
| --- | --- |
| 共享库、静态库 | `/usr/local/lib` |
| 头文件 | `/usr/local/include` |

## 2. 安装 PCL 及相关依赖

点云库（PCL）用于处理激光雷达数据。

### 2.1 确认 ROS 环境

以下软件包名称通过 `$ROS_DISTRO` 选择 ROS 发行版，请在已加载相应 ROS 环境的终端中执行。可先检查当前值：

```bash
echo "$ROS_DISTRO"
```

如果输出为空，先加载所用发行版的 ROS 环境，再继续安装。

### 2.2 安装依赖包

```bash
sudo apt install ros-$ROS_DISTRO-pcl-ros
sudo apt install ros-$ROS_DISTRO-pcl-conversions
sudo apt install libgoogle-glog-dev
```

## 3. 卸载 Livox-SDK2（可选）

仅在需要卸载时执行。以下命令会删除匹配的 SDK 库和头文件，执行前请确认路径及匹配范围。

```bash
sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
sudo rm -rf /usr/local/include/livox_lidar_*
```
