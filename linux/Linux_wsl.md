# WSL 完整使用教程

## WSL 基础管理

### WSL 安装与配置

#### 1. 初始安装
```powershell
# 启用 WSL 功能
wsl --install

# 查看可安装的 Linux 发行版
wsl --list --online

# 安装特定发行版（如 Ubuntu 22.04）
wsl --install -d Ubuntu-22.04
```

#### 2. WSL 版本管理
```powershell
# 查看已安装的 WSL 发行版
wsl --list --verbose

# 设置默认 WSL 版本
wsl --set-default-version 2

# 设置默认发行版
wsl --set-default Ubuntu-22.04

# 查看 WSL 状态
wsl --status
```

#### 3. WSL 备份与恢复
```powershell
# 关闭所有 WSL 实例
wsl --shutdown

# 导出备份
wsl --export Ubuntu-22.04 D:\WSL_Backup\Ubuntu-22.04.tar

# 卸载 WSL
wsl --unregister Ubuntu-22.04

# 从备份导入
wsl --import Ubuntu-22.04 D:\WSL_Ubuntu D:\WSL_Backup\Ubuntu-22.04.tar

# 启动特定 WSL 发行版
wsl -d Ubuntu-22.04
```

---

## USB 设备连接教程

### 1. USBIPD-WIN 安装

#### 方法一：使用 Winget（推荐）
```powershell
# 以管理员身份运行 PowerShell
winget install --interactive --exact dorssel.usbipd-win
```

#### 方法二：手动安装
1. 访问 [USBIPD-WIN GitHub Releases](https://github.com/dorssel/usbipd-win/releases)
2. 下载最新的 `.msi` 安装包
3. 以管理员身份运行安装程序

#### 方法三：Chocolatey
```powershell
choco install usbipd
```

### 2. 验证安装
```powershell
# 检查服务状态
Get-Service usbipd

# 查看版本信息
usbipd --version
```

---

## USB 设备连接完整流程

### 1. 设备连接前准备

#### 在 Windows 端：
```powershell
# 列出所有 USB 设备
usbipd list

# 示例输出：
# BUSID  VID:PID    DEVICE              STATE
# 2-1    1d6b:0002                    Not shared
# 2-2    1d6b:0003                    Not shared
# 2-8    1a86:7523  USB-SERIAL CH340   Not shared
```

#### 在 WSL2 端：
```bash
# 更新系统并安装必要工具
sudo apt update
sudo apt upgrade -y

# 安装 USBIP 工具
sudo apt install linux-tools-generic linux-tools-$(uname -r) hwdata

# 加载必要的内核模块
sudo modprobe usbip-core
sudo modprobe usbip-host
sudo modprobe vhci-hcd

# 验证模块加载
lsmod | grep usbip
```

### 2. 连接 USB 设备

#### 步骤 1：绑定设备（Windows 端）
```powershell
# 普通绑定（推荐先尝试）
usbipd bind --busid <BUSID>

# 如果出现警告，使用强制绑定
usbipd bind --busid <BUSID> --force

# 示例（根据你的 BUSID）：
usbipd bind --busid 2-8 --force
```

#### 步骤 2：连接到 WSL2（Windows 端）
```powershell
# 连接到默认 WSL 发行版
usbipd attach --wsl --busid <BUSID>

# 连接到特定 WSL 发行版
usbipd attach --wsl --busid <BUSID> --distribution <发行版名称>

# 示例：
usbipd attach --wsl --busid 2-8
```

#### 步骤 3：验证连接（WSL2 端）
```bash
# 检查 USB 设备
lsusb

# 查找串口设备
ls /dev/ttyUSB* /dev/ttyACM*

# 设置设备权限
sudo chmod 666 /dev/ttyUSB0

# 永久解决方案：将用户添加到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录或执行：
newgrp dialout
```

### 3. 断开 USB 设备连接

#### 方法一：正常断开
```powershell
# Windows 端断开连接
usbipd detach --busid <BUSID>

# 示例：
usbipd detach --busid 2-8
```

#### 方法二：强制断开
```powershell
# 如果正常断开失败
usbipd detach --busid <BUSID> --force
```

#### 方法三：解除绑定
```powershell
# 完全解除设备绑定
usbipd unbind --busid <BUSID>
```

---

## 实用脚本和自动化

### 1. 快速连接脚本

创建 `connect_usb.ps1`：
```powershell
# 必须以管理员身份运行
param(
    [string]$BusId = "2-8",
    [string]$Distribution = "Ubuntu-22.04"
)

Write-Host "正在连接 USB 设备..." -ForegroundColor Yellow

# 检查设备状态
$devices = usbipd list
if ($devices -match $BusId) {
    Write-Host "找到设备: $BusId" -ForegroundColor Green
    
    # 绑定设备
    usbipd bind --busid $BusId --force
    
    # 连接到 WSL2
    usbipd attach --wsl --busid $BusId --distribution $Distribution
    
    Write-Host "设备连接成功!" -ForegroundColor Green
} else {
    Write-Host "未找到设备: $BusId" -ForegroundColor Red
}
```

### 2. 快速断开脚本

创建 `disconnect_usb.ps1`：
```powershell
param(
    [string]$BusId = "2-8"
)

Write-Host "正在断开 USB 设备..." -ForegroundColor Yellow

usbipd detach --busid $BusId
usbipd unbind --busid $BusId

Write-Host "设备已断开!" -ForegroundColor Green
```

### 3. WSL2 中的设备检查脚本

创建 `check_devices.sh`：
```bash
#!/bin/bash

echo "=== USB 设备检查 ==="
lsusb

echo -e "\n=== 串口设备检查 ==="
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "未找到串口设备"

echo -e "\n=== 设备权限检查 ==="
for device in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$device" ]; then
        echo "$device: $(ls -la $device | cut -d' ' -f1)"
    fi
done
```

---

## 常见问题解决方案

### 1. 权限问题
```bash
# WSL2 中解决权限问题
sudo chmod 666 /dev/ttyUSB0

# 永久解决方案
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
```

### 2. 设备未找到
```powershell
# 重新扫描 USB 设备
usbipd list

# 重启 USBIPD 服务
Restart-Service usbipd
```

### 3. 连接失败
```bash
# WSL2 中重启 USBIP 服务
sudo service usbip restart

# 重新加载内核模块
sudo modprobe -r vhci-hcd
sudo modprobe vhci-hcd
```
---

### WSL2内核源码编译及符号链接创建指南
1.查看当前内核版本

```bash
uname -r
```
2.下载内核源码
进入WSL2内核源码仓库：
https://github.com/microsoft/WSL2-Linux-Kernel
选择与当前内核版本对应的源码分支进行下载。
下载完成后，解压源码包并进入源码目录。
3.安装编译依赖

```bash
sudo apt install build-essential flex bison dwarves libssl-dev libelf-dev cpio qemu-utils
sudo apt install libelf-dev build-essential pkg-config
sudo apt install bison build-essential flex libssl-dev libelf-dev bc
sudo apt-get install libssl-dev
```



4.编译内核

```bash
make KCONFIG_CONFIG=Microsoft/config-wsl && make INSTALL_MOD_PATH="$PWD/modules" modules_install
```

5.创建正确的符号链接
进入编译好的内核文件目录并获取当前路径：
```bash
pwd
```
示例输出：
```plaintext
/home/ubuntu/drivers/WSL2-Linux-Kernel-linux-msft-wsl-6.6.87.2
```
然后创建符号链接：
```bash
# 删除可能存在的错误链接
sudo rm -f /lib/modules/$(uname -r)/build

# 创建指向你内核源码目录的符号链接
# 示例： sudo ln -sf /home/ubuntu/drivers/WSL2-Linux-Kernel-linux-msft-wsl-6.6.87.2 /lib/modules/$(uname -r)/build
sudo ln -sf [你的内核源码目录路径] /lib/modules/$(uname -r)/build


# 验证链接
ls -la /lib/modules/$(uname -r)/build
```
若出现类似如下输出，说明符号链接创建成功：
```plaintext
lrwxrwxrwx 1 root root 62 Oct 19 23:35 /lib/modules/6.6.87.2-microsoft-standard-WSL2/build -> /home/ubuntu/drivers/WSL2-Linux-Kernel-linux-msft-wsl-6.6.87.2
```
