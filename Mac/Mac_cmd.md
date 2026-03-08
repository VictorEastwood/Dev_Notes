# macOS 基本命令速查表
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
open .           # (特有) 在 Finder 中打开当前目录
```
### 包管理（推荐使用 Homebrew）
```bash
# 需先安装 Homebrew，访问 brew.sh 获取安装脚本
brew update         # 更新 Homebrew 自身和软件包列表
brew upgrade        # 升级所有可更新的软件包
brew install <包名> # 安装软件包
brew uninstall <包名> # 卸载软件包
brew search <关键字> # 搜索软件包
brew list           # 列出已安装的软件包
```
### 文件压缩与解压
```bash
tar -cvf <压缩包.tar> <文件/目录>   # 创建 tar 包
tar -xvf <压缩包.tar>               # 解压 tar 包
tar -czvf <压缩包.tar.gz> <文件/目录>   # 创建 tar.gz 包
tar -xzvf <压缩包.tar.gz>            # 解压 tar.gz 包
zip <压缩包.zip> <文件/目录>       # 创建 zip 压缩包
unzip <压缩包.zip>                 # 解压 zip 包
# macOS 自带解压工具，也可直接双击解压
```
### 系统管理
```bash
sudo <命令>         # 使用管理员权限执行命令
sudo shutdown -h now     # 立即关机
sudo reboot             # 重启系统
whoami             # 显示当前用户
date               # 显示当前日期和时间
sw_vers            # (特有) 显示 macOS 系统版本信息
caffeinate         # (特有) 防止系统休眠（按 Ctrl+C 结束）
```
### 文件权限和所有权
```bash
chmod <权限> <文件>  # 更改文件权限
chown <用户>:<组> <文件>   # 更改文件所有者
chgrp <组> <文件>   # 更改文件组
umask             # 显示当前权限掩码
xattr -l <文件>     # (特有) 查看文件的扩展属性
```
### 编辑文件
```bash
nano <文件>        # 使用 nano 编辑文件（系统自带）
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
# macOS 无 free 命令，可使用 top 或 vm_stat 查看内存
vm_stat          # 显示虚拟内存统计信息
uptime           # 显示系统启动时间及负载
```
### 网络操作
```bash
ping <IP/域名>       # 检查网络连通性
ifconfig           # 查看网络接口的配置（类似 Linux 的 ip a）
netstat -an        # 查看当前的网络连接和端口
curl <URL>         # 获取URL内容（系统自带）
curl -O <URL>      # 下载文件（类似 wget）
traceroute <IP/域名> # 跟踪网络路由
networksetup -listallhardwareports # (特有) 列出所有网络硬件端口
```
### 用户管理
```bash
# macOS 用户管理多通过“系统设置”完成，命令行工具如下：
id <用户名>           # 查看用户的ID和组信息
dscl . list /Users   # 列出本地所有用户
passwd               # 修改当前用户密码
# 创建用户推荐使用 sysadminctl，比 useradd 更适配 macOS
```
### 进程管理
```bash
kill <PID>         # 结束指定进程
killall <进程名>   # 结束所有指定进程
ps aux | grep <进程名>  # 查找进程
pkill <进程名>     # 按名称结束进程
```
### 磁盘管理
```bash
diskutil list      # (特有) 列出所有磁盘和分区（类似 fdisk -l）
mount <设备> <挂载点>   # 挂载设备
umount <挂载点>        # 卸载设备
diskutil info <磁盘标识> # 显示指定磁盘的详细信息
```
### 环境变量
```bash
export <变量名>=<值>   # 设置环境变量（临时）
echo $<变量名>        # 查看环境变量的值
# macOS 默认使用 zsh，永久变量通常配置在 ~/.zshrc 文件中
```
### 查看日志
```bash
log show           # (特有) 访问系统日志数据库（比 dmesg 更强大）
log stream         # 实时显示系统日志
tail -f <日志文件>  # 实时查看特定日志文件
```
### macOS 特有实用工具
```bash
pbcopy < <文件>    # 将文件内容复制到剪切板
pbpaste            # 将剪切板内容粘贴到终端
say <文本>         # 语音朗读文本
mdfind <关键字>    # 使用 Spotlight 搜索文件
screencapture -c   # 截屏并保存到剪切板
```
