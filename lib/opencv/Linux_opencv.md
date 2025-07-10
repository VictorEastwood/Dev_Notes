# Ubuntu 环境下Opencv c++ with opencv-contrib配置
---
注意，在安装opencv之前，需要配置好基础开发环境，具体可以查看[Linux基础开发环境配置](Linux_development.md)
## 1. 下载源码
[https://opencv.org/releases/](https://opencv.org/releases/)

[https://github.com/opencv/opencv_contrib/tags](https://github.com/opencv/opencv_contrib/tags)
（opencv-contrib 文件夹放在opencv 文件夹里面）

## 2. 安装依赖项

```bash

sudo apt install -y libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

如果出现以下情况 `Unable to locate package libjasper-dev`，执行以下命令:

```bash
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev
```
## 3.编译
新建并进入build文件夹

```bash
mkdir build
cd build
```

然后在该路径下cmake（这里版本号需要改成实际值）

```bash
sudo cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.8.0/modules ..
```
查看线程数，线程为4，因此数字为4

```bash
nproc
```
编译如果是4核心就写4 是16核心就写16

```bash
sudo make -j16

```
## 4.安装Opencv

```bash
sudo make install
```
## 5.配置Opencv
打开opencv配置文档修改动态库。

```bash
sudo gedit /etc/ld.so.conf
```

在文档末尾添加:
```
include /usr/local/lib
```

运行加载配置：

```bash
sudo ldconfig
```

运行

```bash
sudo gedit /etc/bash.bashrc
```

打开后在文档末尾添加

```bash
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH
```

保存退出，在终端输入

```bash
source /etc/bash.bashrc
```

新建终端输入

```bash
sudo updatedb
```

## 6.测试安装

验证查看版本号

```bash
pkg-config --modversion opencv4
```

查看opencv的链接库

```bash
pkg-config --libs opencv4 
```

