# Git相关操作

## 1. 安装Git
输入以下命令安装Git：
```bash
sudo apt install -y git
```
查看Git版本：
```bash
git --version
```

## 2. 配置Git
设置用户名和邮箱：
```bash
git config --global user.name "Shidong Wu"
git config --global user.email "epsilon5400@gmail.com"
```

## 3. 创建Git仓库
在当前目录下创建一个新的Git仓库：
```bash
git init
```
## 4. 克隆Git仓库
克隆一个远程Git仓库到本地：
```bash
git clone <repository-url>
```

## 5. 添加文件到Git仓库
在当前目录下添加文件到Git仓库：
```bash
git add <file>
```
添加所有文件到Git仓库：
```bash
git add .
```
## 6. 提交更改
提交更改到Git仓库：
```bash
git commit -m "Your commit message"
```
## 7. 查看Git状态
查看当前Git仓库的状态：
```bash
git status
```
## 8. 查看Git日志
查看Git提交历史：
```bash
git log
```
