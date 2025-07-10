# KDL库

## 安装KDL库
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
