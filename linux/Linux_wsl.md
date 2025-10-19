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

### 4. 驱动问题
```bash
# 安装常见串口驱动
sudo apt install linux-modules-extra-$(uname -r)

# 加载常见串口芯片驱动
sudo modprobe ch341
sudo modprobe ftdi_sio
sudo modprobe pl2303
sudo modprobe cp210x
```

---

## 高级使用技巧

### 1. 批量设备管理
```powershell
# 查看所有可用的 USB 设备
usbipd list

# 批量断开所有设备
usbipd list | ForEach-Object { 
    if ($_ -match "(\d+-\d+).*Shared") { 
        usbipd detach --busid $matches[1] 
    } 
}
```

### 2. 持久化配置
```bash
# 在 WSL2 中设置自动加载模块
echo -e "usbip-core\nusbip-host\nvhci-hcd" | sudo tee -a /etc/modules

# 设置自动权限
sudo tee /etc/udev/rules.d/99-serial.rules << EOF
KERNEL=="ttyUSB[0-9]*", MODE="0666"
KERNEL=="ttyACM[0-9]*", MODE="0666"
EOF

# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. 性能优化
```bash
# 在 WSL2 中创建配置文件
sudo tee /etc/wsl.conf << EOF
[automount]
enabled = true
mountFsTab = true

[network]
generateHosts = true
generateResolvConf = true

[interop]
enabled = true
appendWindowsPath = true
EOF
```

---

## 使用示例

### 示例 1：连接 Arduino
```powershell
# Windows 端
usbipd bind --busid 2-8 --force
usbipd attach --wsl --busid 2-8

# WSL2 端
ls /dev/ttyUSB0
# 输出: /dev/ttyUSB0
```

### 示例 2：连接多个设备
```powershell
# 连接多个 USB 转串口设备
usbipd bind --busid 2-8 --force
usbipd bind --busid 3-5 --force
usbipd attach --wsl --busid 2-8
usbipd attach --wsl --busid 3-5
```

### 示例 3：ROS 2 串口通信测试
```bash
# 在 WSL2 中测试串口通信
cd ~/ros2_serial_ws
source install/setup.bash
ros2 run serial_example wjwood_serial_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

---
