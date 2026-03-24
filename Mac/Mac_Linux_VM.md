# Mac 安装 Linux 虚拟机
## 1. 下载 Arm64 版镜像
> **注意**：请根据您的 Mac 芯片架构（Intel 或 Apple Silicon）选择对应的 Linux 镜像版本进行下载。
## 2. 基础环境配置
成功安装系统后，需要进行基础的网络和工具配置。
### 2.1 查看系统信息
查看内核及系统架构信息：
```bash
uname -a
```
### 2.2 安装网络工具
默认系统可能未安装 `ifconfig` 命令，需要手动安装 `net-tools`：
```bash
apt install net-tools
```
### 2.3 网络连接与 SSH
1. **查看 IP 地址**：
   ```bash
   ifconfig
   ```
2. **SSH 远程连接**（在宿主机终端执行）：
   ```bash
   ssh 用户名@IP地址
   ```
---
## 3. 安装桌面环境
如果安装的是服务器版本，通常没有图形界面，需手动安装。
### 3.1 更新系统软件包
在安装桌面环境前，建议先更新系统。
**执行更新命令：**
```bash
sudo apt update
sudo apt upgrade
```
**常见报错处理：**
若更新过程中出现以下错误：
> `dpkg was interrupted, you must manually run 'dpkg --configure -a' to correct the problem.`
**解决方案：**
请执行以下命令修复，然后重新更新：
```bash
sudo dpkg --configure -a
sudo apt upgrade
```
### 3.2 安装 Ubuntu Desktop
执行以下命令安装图形界面：
```bash
sudo apt install ubuntu-desktop
```
### 3.3 启用图形界面
安装完成后，设置系统启动目标为图形模式：
```bash
sudo systemctl set-default graphical.target
```
### 3.4 重启系统
完成所有配置后，重启系统以进入桌面环境：
```bash
sudo reboot
```
