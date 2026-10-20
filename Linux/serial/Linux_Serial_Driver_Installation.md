# Linux 串口驱动安装与配置

## 1. Linux 串口配置

在 Linux 系统中，与串口相关的冲突问题通常涉及 `brltty`（Braille TTY）包。`brltty` 是一个为视障人士提供盲文显示支持的服务，但它可能会自动占用某些串口设备（如 `/dev/ttyS*` 或 `/dev/ttyUSB*`），导致其他应用程序无法访问这些串口。

### 1.1 解决 `brltty` 冲突问题

1. **检查是否安装了 `brltty`**：

   ```bash
   dpkg -l | grep brltty  # Debian/Ubuntu
   ```

2. **停止并卸载 `brltty`**：

   ```bash
   sudo systemctl stop brltty-udev.service      # 停止服务
   sudo systemctl disable brltty-udev.service   # 禁用开机自启
   sudo apt remove brltty                      # Debian/Ubuntu
   ```

3. **检查串口是否可用**：

   卸载后，重新插拔串口设备，或运行：

   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

   然后尝试访问串口（如使用 `minicom` 或 `screen`）。

## 2. CH340/CH343 驱动安装和配置

### 2.1 查看现有 CH340 驱动

```bash
ls /lib/modules/$(uname -r)/kernel/drivers/usb/serial
```

一般能看到以下文件：

```text
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
sudo rm -rf /lib/modules/$(uname -r)/kernel/drivers/usb/serial/ch341.ko
```

### 2.2 禁用 `cdc-acm` 驱动

1. **临时卸载驱动**（立即删除 `/dev/ttyACM0`）：

   ```bash
   sudo rmmod cdc_acm
   ```

2. **验证设备节点已消失**：

   ```bash
   ls /dev/ttyACM0
   ```

   此时提示“没有该文件或目录”属于正常现象。

3. **加入黑名单**，避免重启后自动加载：

   ```bash
   echo "blacklist cdc_acm" | sudo tee /etc/modprobe.d/blacklist-acm.conf
   ```

4. **强化禁用**，防止被其他模块依赖触发加载：

   ```bash
   echo "install cdc_acm /bin/false" | sudo tee -a /etc/modprobe.d/blacklist-acm.conf
   ```

5. **更新内核镜像缓存**，确保重启后规则生效：

   ```bash
   sudo update-initramfs -u
   ```

6. **重启系统**，使配置完全生效：

   ```bash
   sudo reboot
   ```

### 2.3 安装官方 CH34X 驱动

#### CH343 驱动

下载并编译安装：

```bash
git clone https://github.com/WCHSoftGroup/ch343ser_linux.git
cd ch343ser_linux/driver
make
sudo make install
```

#### CH341 驱动

下载并编译安装：

```bash
git clone https://github.com/WCHSoftGroup/ch341ser_linux.git
cd ch341ser_linux/driver
make
sudo make install
```

#### 常见报错：缺少 `gcc-12`

如果编译时显示如下报错：

```text
make -C /lib/modules/6.8.0-138-generic/build  M=/ch343ser_linux/driver modules
make[1]: Entering directory '/usr/src/linux-headers-6.8.0-138-generic'
warning: the compiler differs from the one used to build the kernel
  The kernel was built by: x86_64-linux-gnu-gcc-12 (Ubuntu 12.3.0-1ubuntu1~22.04.3) 12.3.0
  You are using:
  CC [M]  /ch343ser_linux/driver/ch343.o
/bin/sh: 1: gcc-12: not found
make[3]: *** [scripts/Makefile.build:243: /ch343ser_linux/driver/ch343.o] Error 127
make[2]: *** [/usr/src/linux-headers-6.8.0-138-generic/Makefile:1946: /ch343ser_linux/driver] Error 2
make[1]: *** [Makefile:240: __sub-make] Error 2
make[1]: Leaving directory '/usr/src/linux-headers-6.8.0-138-generic'
make: *** [Makefile:13: all] Error 2
```

则需要安装 `gcc-12` 编译器：

```bash
sudo apt install gcc-12
```

### 2.4 验证驱动安装

安装完毕后，插拔 CH340 设备，查看是否识别：

```bash
ls /dev/
```

如果出现类似以下信息，说明设备已经被识别：

```text
ttyCH341USB0
```

### 2.5 配置串口访问权限

若识别成功，则可以添加用户到 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

## 3. ROS 2 串口库教程

在 ROS 2 中使用串口库进行串口通信，可以使用 `serial` 库。

### 3.1 编译 `serial` 库

```bash
git clone https://github.com/ZhaoXiangBox/serial.git
cd serial
mkdir build
cd build
cmake ..
make
```

### 3.2 安装 `serial` 库

```bash
sudo make install
sudo ldconfig
```
