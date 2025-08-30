# Linux 基础工作配置

>本章将介绍Linux系统的基本安装与初始工作配置，确保你能实现一个良好的Linux工作环境。
>本章内容适用于Ubuntu等Debian系Linux发行版。
## 安装系统（双系统）
### 下载镜像
- [Ubuntu](https://ubuntu.com/download/desktop)
### 制作启动盘
- [Rufus](https://rufus.ie/)
### windows下删除原有分区
注意：删除分区会导致数据丢失，请提前备份数据以下命令中的1是你的linux分区号，可以通过diskpart命令查看
```shell
diskpart
list disk
select disk 1
list partition
clean
```
### 更换软件源
我们使用小鱼提供的一键安装脚本来更换软件源，脚本如下：
```shell
wget http://fishros.com/install -O fishros && . fishros
```
## 安装必要软件：
### 更新软件包
```shell
sudo apt update
sudo apt upgrade
```
### 软件安装
#### 安装搜狗输入法
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
9. 或者下载普通谷歌输入法
```bash
sudo apt-get install fcitx-googlepinyin -y
```

#### 修改系统字体
默认英文系统下中文字体很奇怪需要做以下修改：
```bash
sudo gedit /etc/fonts/conf.avail/64-language-selector-prefer.conf
```
将所有SC(Simplify Chinese)开头的置顶。

#### 安装Clash for Windows Linux版
1. 下载软件包：
```shell
wget https://github.com/clashdownload/Clash_for_Windows/releases/download/0.20.39/Clash.for.Windows-0.20.39-x64-linux.tar.gz
```
如果 wget 下载不了，就把下载链接复制到浏览器下载。

1. 运行软件
然后找到你下载的安装包，解压提取，打开文件夹，里面有一个 cfw 文件，双击就是 Clash 了。如果不行，请在该文件夹内打开终端，使用./cfw命令执行它。
1. 把网络代理改成手动，按照如下设置：
- HTTP代理 `127.0.0.1` `7890`
- HTTPS代理 `127.0.0.1` `7890`
- FTP代理 `      ` `0`
- Socks主机(s)`127.0.0.1` `7890`
到浏览器测试YouTube，看看是否可以打开: https://www.youtube.com/

## 其他设置
### grub引导界面美化
进入网站选择你需要的主题并下载
https://www.gnome-look.org/browse?cat=109&ord=latest

使用 root 用户做如下操作
```bash
sudo -i
```
创建主题目录
```bash
cd /boot/grub
mkdir themes
cd themes
```
下载并移动主题
```bash
cd ~/Downloads
# 假设你下载的主题是 your_themes.tar.gz 或 your_themes.zip
mv <your_themes> /boot/grub/themes/
```
解压主题
```bash
tar -zxvf <your_themes>.tar.gz
unzip <your_themes>.zip
```
设置主题
```bash
cd <your_themes>
./install.sh
```
安装完之后重启即可
```bash
reboot
```


