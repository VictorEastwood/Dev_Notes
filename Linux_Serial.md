# Linux 串口

## 1. 查看CH340驱动

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
上面可以看到含有ch341.ko文件，系统自带的版本比较老，删除掉：
```bash
cd /lib/modules/$(uname -r)/kernel/drivers/usb/serial
sudo rm -rf ch341.ko
```

## 2.检查硬件设备
插上CH340设备，查看是否识别：
```bash
lsusb
```
如果出现类似以下信息，说明设备已经被识别：
```
Bus 003 Device 012: ID 1a86:7523 QinHeng Electronics CH340 serial converter
```

## 3.安装新驱动
官网下载链接：[CH340驱动](https://www.wch.cn/download/CH341SER_LINUX_ZIP.html)
下载后解压，进入解压目录，执行：
```bash
unzip CH341SER_LINUX.ZIP
cd CH341SER_LINUX
```
可以查阅README文件，里面有详细的安装说明:
```
ch341 Linux串口驱动
说明
这是一个用于CH340、CH341等USB转UART芯片的USB串口驱动程序。虽然CH341支持多种工作模式，但此驱动仅支持其串口模式。

实际上，自Linux主线内核2.6.24版本以来，ch341串口驱动就已经内置于内核中，位于drivers/usb/serial/ch341.c。遗憾的是，内置驱动无法及时更新且不能支持芯片的所有功能。因此我们建议客户使用此驱动程序。

安装步骤
打开"终端"
切换到"driver"目录
使用make命令编译驱动，如果成功会生成ch341.ko模块
输入sudo make load或sudo insmod ch341.ko来动态加载驱动
输入sudo make unload或sudo rmmod ch341.ko来卸载驱动
输入sudo make install使驱动程序永久生效
输入sudo make uninstall来移除驱动
您可以参考以下链接获取uart应用程序，可以使用gcc或交叉编译器进行编译： https://github.com/WCHSoftGroup/tty_uart
使用前提
在驱动工作之前，请确保USB设备已正确插入并正常工作。您可以使用shell命令lsusb或dmesg来确认，这些设备的USB VID为[1a86]。您可以在ch341.c中定义的ID表中查看所有ID。

如果设备工作正常，驱动程序将在dev目录下创建名为ttyCH341USBx的tty设备。

注意
如有任何问题，请发送邮件至：tech@wch.cn
```
安装完毕后，插拔CH340设备，查看是否识别：
```bash
ls /dev/
```
如果出现类似以下信息，说明设备已经被识别：
``` 
ttyCH341USB0
```
## 4.测试串口
安装串口调试工具：
```bash
sudo apt-get install cutecom
```
打开串口调试工具
```bash
sudo cutecom
```
选择串口设备ttyCH341USB0，设置波特率为115200，数据位8，校验位None，停止位1，流控None。
将CH340设备的TXD和RXD短接，输入字符并按回车，如果能够回显，说明串口正常。
