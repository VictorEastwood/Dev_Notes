# Linux 串口

在 Linux 系统中，与串口（Serial Port）相关的冲突问题通常涉及 `brltty`（Braille TTY）包。`brltty` 是一个为视障人士提供盲文显示支持的服务，但它可能会自动占用某些串口设备（如 `/dev/ttyS*` 或 `/dev/ttyUSB*`），导致其他应用程序无法访问这些串口。

---

## 解决 brltty 冲突问题

### 检查和卸载 `brltty`

1. **检查是否安装了 `brltty`**：
   ```bash
   dpkg -l | grep brltty  # Debian/Ubuntu
   rpm -qa | grep brltty  # RHEL/CentOS/Fedora
   ```

2. **停止并卸载 `brltty`**：
   ```bash
   sudo systemctl stop brltty-udev.service      # 停止服务
   sudo systemctl disable brltty-udev.service   # 禁用开机自启
   sudo apt remove brltty                        # Debian/Ubuntu
   sudo yum remove brltty                        # RHEL/CentOS
   sudo dnf remove brltty                        # Fedora
   ```

3. **检查串口是否可用**：
   卸载后，重新插拔串口设备或运行：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
   然后尝试访问串口（如使用 `minicom` 或 `screen`）。

### 替代方案（如果不希望完全卸载）

如果仍需要盲文支持，可以修改 `brltty` 的配置，使其不占用特定串口：
- 编辑 `/etc/brltty.conf`，注释掉或移除相关串口设备
- 或者使用 `udev` 规则阻止 `brltty` 绑定特定设备（如 `/dev/ttyUSB0`）

### 注意事项

- 卸载 `brltty` 可能会影响视障用户的辅助功能，请确保系统无此类需求后再操作
- 如果问题依旧，检查是否有其他服务（如 `ModemManager`）占用串口，并参考相关方法解决

如需进一步排查，可结合 `lsof /dev/ttyS*` 或 `dmesg` 查看具体占用进程。

---

## CH340 驱动安装和配置

### 1. 查看 CH340 驱动

```bash
ls /lib/modules/$(uname -r)/kernel/drivers/usb/serial
```

一般能看到下面等文件：
```
aircable.ko         io_ti.ko        navman.ko        ti_usb_3410_5052.ko
ark3116.ko          ipaq.ko         omninet.ko       upd78f0730.ko
belkin_sa.ko        ipw.ko          opticon.ko       usb_debug.ko
ch341.ko            ir-usb.ko       option.ko        usbserial.ko
cp210x.ko           iuu_phoenix.ko  oti6858.ko       usb-serial-simple.ko
cyberjack.ko        keyspan.ko      pl2303.ko        usb_wwan.ko
cypress_m8.ko       keyspan_pda.ko  qcaux.ko         visor.ko
digi_acceleport.ko  kl5kusb105.ko   qcserial.ko      whiteheat.ko
empeg.ko            kobil_sct.ko    quatech2.ko      wishbone-serial.ko
f81232.ko           mct_u232.ko     safe_serial.ko   xr_serial.ko
f81534.ko           metro-usb.ko    sierra.ko        xsens_mt.ko
ftdi_sio.ko         mos7720.ko      spcp8x5.ko
garmin_gps.ko       mos7840.ko      ssu100.ko
io_edgeport.ko      mxuport.ko      symbolserial.ko
```

上面可以看到含有 `ch341.ko` 文件，系统自带的版本比较老，删除掉：
```bash
cd /lib/modules/$(uname -r)/kernel/drivers/usb/serial
sudo rm -rf ch341.ko
```

### 2. 检查硬件设备

插上 CH340 设备，查看是否识别：
```bash
lsusb
```

如果出现类似以下信息，说明设备已经被识别：
```
Bus 003 Device 012: ID 1a86:7523 QinHeng Electronics CH340 serial converter
```

### 3. 安装新驱动

官网下载链接：[CH340驱动](https://www.wch.cn/download/CH341SER_LINUX_ZIP.html)

下载后解压，进入解压目录：
```bash
unzip CH341SER_LINUX.ZIP
cd CH341SER_LINUX
```

**安装步骤：**

1. **打开终端并切换到驱动目录**：
   ```bash
   cd driver
   ```

2. **编译驱动**：
   ```bash
   make                    # 编译驱动，成功会生成ch341.ko模块
   ```

3. **安装驱动**：
   ```bash
   sudo make install       # 安装驱动（永久生效）
   ```

4. **加载驱动**：
   ```bash
   sudo make load          # 动态加载驱动
   # 或者使用: sudo insmod ch341.ko
   ```

5. **卸载驱动**（如需要）：
   ```bash
   sudo make unload        # 卸载驱动
   # 或者使用: sudo rmmod ch341.ko
   sudo make uninstall     # 移除驱动
   ```

**参考链接：**  
您可以参考以下链接获取uart应用程序，可以使用gcc或交叉编译器进行编译：  
https://github.com/WCHSoftGroup/tty_uart

安装完毕后，插拔 CH340 设备，查看是否识别：
```bash
ls /dev/
```

如果出现类似以下信息，说明设备已经被识别：
```
ttyCH341USB0
```

### 4. 测试串口

安装串口调试工具：
```bash
sudo apt-get install cutecom
```
添加用户组

```bash
sudo usermod -aG dialout $USER
```
重启系统或重新登录以使用户组更改生效。

打开串口调试工具：
```bash
sudo cutecom
```

选择串口设备 `ttyCH341USB0`，设置波特率为 115200，数据位 8，校验位 None，停止位 1，流控 None。

将 CH340 设备的 TXD 和 RXD 短接，输入字符并按回车，如果能够回显，说明串口正常。

---

## 常用串口工具

除了 `cutecom`外，还可以使用以下工具进行串口调试：

- `minicom`：命令行串口工具
- `screen`：多用途终端程序，也可用于串口通信
- `picocom`：轻量级串口工具

## ros2使用串口库教程

在 ROS2 中使用串口库进行串口通信，可以使用 `serial` 库。以下是一个简单的教程，介绍如何在 ROS2 中使用 `serial` 库进行串口通信。
### 1. 安装 `serial` 库
首先，确保已经安装了 `serial` 库。可以使用以下命令进行安装：
```bash
sudo apt install ros-${ROS_DISTRO}-serial-driver
sudo apt install ros-${ROS_DISTRO}-asio-cmake-module
sudo apt install ros-${ROS_DISTRO}-io-context
```
### 2.安装serial库依赖
在使用 `serial` 库之前，需要安装一些依赖项。可以使用以下命令进行安装：
```bash
git clone https://github.com/ZhaoXiangBox/serial.git
cd serial
mkdir build
cd build
cmake ..
make
sudo make install
```
