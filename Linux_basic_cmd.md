# Linux 基本命令速查表
---
### 文件和目录操作
```bash
ls               # 列出当前目录下的文件和文件夹
ls -l            # 列出详细信息（权限、大小等）
ls -a            # 显示所有文件（包括隐藏文件）
cd <目录>        # 进入指定目录
pwd              # 显示当前工作目录的路径
mkdir <目录>     # 创建目录
rmdir <目录>     # 删除空目录
rm <文件>        # 删除文件
rm -r <目录>     # 删除目录及其中所有内容
cp <源文件> <目标文件>    # 复制文件
mv <源文件> <目标文件>    # 移动文件或重命名文件
touch <文件>     # 创建空文件或更新文件时间戳
find <路径> -name <文件名>   # 查找文件
cat <文件>       # 查看文件内容
less <文件>      # 分页显示文件内容
head <文件>      # 显示文件开头内容
tail <文件>      # 显示文件结尾内容
```
### 包管理（基于不同的Linux发行版）
```bash
# Debian/Ubuntu系
apt update         # 更新软件包列表
apt upgrade        # 升级所有可更新的软件包
apt install <包名> # 安装软件包
apt remove <包名>  # 删除软件包

# CentOS/RHEL系
yum update         # 更新软件包
yum install <包名> # 安装软件包
yum remove <包名>  # 删除软件包
```
### 文件压缩与解压
```bash
tar -cvf <压缩包.tar> <文件/目录>   # 创建 tar 包
tar -xvf <压缩包.tar>               # 解压 tar 包
tar -czvf <压缩包.tar.gz> <文件/目录>   # 创建 tar.gz 包
tar -xzvf <压缩包.tar.gz>            # 解压 tar.gz 包
zip <压缩包.zip> <文件/目录>       # 创建 zip 压缩包
unzip <压缩包.zip>                 # 解压 zip 包
```
### 系统管理
```bash
sudo <命令>         # 使用管理员权限执行命令
shutdown -h now     # 立即关机
reboot             # 重启系统
whoami             # 显示当前用户
date               # 显示当前日期和时间
```
### 文件权限和所有权
```bash
chmod <权限> <文件>  # 更改文件权限
chown <用户>:<组> <文件>   # 更改文件所有者
chgrp <组> <文件>   # 更改文件组
umask             # 显示当前权限掩码
```
### 编辑文件
```bash
nano <文件>        # 使用 nano 编辑文件
vim <文件>         # 使用 vim 编辑文件
```
### 系统监控
```bash
top              # 实时查看系统资源（CPU、内存、进程等）使用情况
htop             # 更友好的 top，需安装
ps               # 查看当前进程
ps aux           # 查看所有进程
df -h            # 查看磁盘空间使用情况
du -sh <目录>    # 查看目录或文件的大小
free -h          # 查看内存使用情况
uptime           # 显示系统启动时间及负载
```
### 网络操作
```bash
ping <IP/域名>       # 检查网络连通性
ifconfig           # 查看网络接口的配置
ip a               # 查看所有网络接口的IP地址
netstat -tuln      # 查看当前的网络连接
ss -tuln           # 更现代的 netstat
curl <URL>         # 获取URL内容
wget <URL>         # 下载文件
traceroute <IP/域名> # 跟踪网络路由
```
### 用户管理
```bash
useradd <用户名>       # 添加新用户
usermod -aG <组名> <用户名>  # 将用户添加到组
passwd <用户名>       # 修改用户密码
groupadd <组名>       # 创建新组
groupdel <组名>       # 删除组
id <用户名>           # 查看用户的ID和组信息
```
### 进程管理
```bash
kill <PID>         # 结束指定进程
killall <进程名>   # 结束所有指定进程
ps aux | grep <进程名>  # 查找进程
```
### 磁盘管理
```bash
mount <设备> <挂载点>   # 挂载设备
umount <挂载点>        # 卸载设备
fdisk -l            # 查看硬盘分区情况
lsblk               # 查看设备和分区
```
### 环境变量
```bash
export <变量名>=<值>   # 设置环境变量
echo $<变量名>        # 查看环境变量的值
```
### 查看日志
```bash
dmesg              # 查看内核日志
tail -f <日志文件>  # 实时查看日志文件
```

