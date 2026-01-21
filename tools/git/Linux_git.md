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

## 添加远程仓库
添加远程仓库到本地Git仓库：
```bash
git remote add origin <remote-repository-url>
```
## 9. 配置 SSH 密钥（免密克隆和推送）

### 9.1 检查是否已有 SSH 密钥
```bash
ls -al ~/.ssh
```
如果看到 `id_rsa` 和 `id_rsa.pub`（或 `id_ed25519` 和 `id_ed25519.pub`），说明已有密钥对。

### 9.2 生成新的 SSH 密钥（如果没有）
```bash
# 推荐使用 ed25519 算法
ssh-keygen -t ed25519 -C "epsilon5400@gmail.com"

# 或使用 RSA 算法
ssh-keygen -t rsa -b 4096 -C "epsilon5400@gmail.com"
```
按提示设置保存路径和密码（可直接按回车使用默认值）。

### 9.3 查看并复制公钥
```bash
# 如果是 ed25519
cat ~/.ssh/id_ed25519.pub

# 如果是 rsa
cat ~/.ssh/id_rsa.pub
```
复制以 `ssh-ed25519` 或 `ssh-rsa` 开头的内容。

### 9.4 将公钥添加到 GitHub
1. 登录 GitHub → 点击头像 → **Settings**
2. 左侧选择 **SSH and GPG keys**
3. 点击 **New SSH key**
4. 输入 Title（如 “My Laptop”）并粘贴公钥内容
5. 点击 **Add SSH key**

### 9.5 测试 SSH 连接
```bash
ssh -T git@github.com
```
看到 `Hi 用户名! You've successfully authenticated...` 表示成功。
