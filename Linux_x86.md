# Linux 基础配置（x86）
---
## 1.安装系统（双系统）
### 1.1 下载镜像
- [Ubuntu](https://ubuntu.com/download/desktop)
### 1.2 制作启动盘
- [Rufus](https://rufus.ie/)
### 1.3 windows下删除原有分区
注意：删除分区会导致数据丢失，请提前备份数据以下命令中的1是你的linux分区号，可以通过diskpart命令查看
```shell
diskpart
list disk
select disk 1
list partition
clean
```
### 1.4 更换软件源
我们使用小鱼提供的一键安装脚本来更换软件源，脚本如下：
```shell
wget http://fishros.com/install -O fishros && . fishros
```
## 2.安装必要软件：
### 2.1 更新软件包
```shell
sudo apt update
sudo apt upgrade
```
### 2.2 软件安装
#### 2.2.1安装搜狗输入法
安装教程：
1. 更新源
   在终端执行 `sudo apt update`
2. 安装fcitx输入法框架
   在终端输入 `sudo apt install fcitx`
3. 设置fcitx为系统输入法
   点击左下角菜单选择语言支持，将语言选择为fcitx
4. 设置fcitx开机自启动
   在终端执行 `sudo cp /usr/share/applications/fcitx.desktop /etc/xdg/autostart/`
5. 卸载系统ibus输入法框架
   在终端执行 `sudo apt purge ibus`
6. 安装搜狗输入法
   在官网https://shurufa.sogou.com/linux 下载搜狗输入法安装包，并安装，安装命令 `sudo dpkg -i 安装包名`
7. 安装输入法依赖
      在终端执行：
      ```bash
      sudo apt install libqt5qml5 libqt5quick5 libqt5quickwidgets5 qml-module-qtquick2
      sudo apt install libgsettings-qt1
      ```
8. 重启电脑
   
#### 2.2.2安装Clash for Windows Linux版
1. 下载软件包：
```shell
wget https://github.com/clashdownload/Clash_for_Windows/releases/download/0.20.39/Clash.for.Windows-0.20.39-x64-linux.tar.gz
```
如果 wget 下载不了，就把下载链接复制到浏览器下载。

2. 运行软件
然后找到你下载的安装包，解压提取，打开文件夹，里面有一个 cfw 文件，双击就是 Clash 了。如果不行，请在该文件夹内打开终端，使用./cfw命令执行它。
3. 把网络代理改成手动，按照如下设置：
- HTTP代理 `127.0.0.1` `7890`
- HTTPS代理 `127.0.0.1` `7890`
- FTP代理 `      ` `0`
- Socks主机(s)`127.0.0.1` `7890`
到浏览器测试YouTube，看看是否可以打开: https://www.youtube.com/



#### 2.2.3 常用软件安装
类似于安卓系统的apk安装方式，Linux系统也有自己的安装方式，那就是.deb文件安装方式。下面是一些常用软件的.deb文件下载地址和安装方式：
| 软件 | 下载地址 |
| --- | --- |
| Chrome | https://www.google.cn/chrome/ |
| VSCode | https://code.visualstudio.com/ |
| QQ | https://im.qq.com/linuxqq/index.shtml |
| 飞书 | https://www.feishu.cn/download |
| 腾讯文档 | https://docs.qq.com/home/download |
| ToDesk | https://www.todesk.com/ |
| 百度网盘 | https://pan.baidu.com/download |

可以进入(quick_install)一键安装常用软件
```shell
sudo dpkg -i *.deb
sudo apt update
sudo apt upgrade
```
---
