# Apple Container 完整配置指南

本指南详细介绍了从零开始在 Apple Silicon Mac 上配置 Apple Container 并实现 X11 图形界面转发的完整流程。

---

## 一、安装与基础配置

### 1.1 安装 Apple Container

Apple Container 是 Apple 官方开源的容器化工具，专为 Apple Silicon 优化。

**安装步骤：**

1. 从 GitHub 官方仓库下载 `.pkg` 安装包
   - 访问：https://github.com/apple/container
   - 下载最新版本的 `.pkg` 文件

2. 双击 `.pkg` 文件，按照安装向导完成安装

3. 安装完成后，启动容器服务：
   ```bash
   container system start
   ```

4. 验证服务状态：
   ```bash
   container system status
   ```

---

### 1.2 拉取 Ubuntu 22.04 镜像

```bash
# 拉取 Ubuntu 22.04 官方镜像
container pull ubuntu:22.04
```

---

### 1.3 创建并启动容器

```bash
# 创建并启动容器（<your_container_name> 替换为实际名称）
container run -dit --name <your_container_name> ubuntu:22.04

# 查看容器列表
container ls -a

# 启动已停止的容器
container start <your_container_name>

# 进入容器终端
container exec -it <your_container_name> /bin/bash
```

**参数说明：**
| 参数 | 说明 |
|------|------|
| `-d` | 后台运行 |
| `-i` | 保持 STDIN 打开（交互式） |
| `-t` | 分配伪终端 |
| `--name` | 指定容器名称 |

---

### 1.4 容器基本操作命令

| 操作 | 命令 | 说明 |
|------|------|------|
| 查看运行容器 | `container ls` | 仅显示运行中的容器 |
| 查看所有容器 | `container ls -a` | 包括停止的容器 |
| 停止容器 | `container stop <名称>` | 停止运行中的容器 |
| 启动容器 | `container start <名称>` | 启动已停止的容器 |
| 删除容器 | `container rm <名称>` | 删除容器（需先停止） |
| 进入容器 | `container exec -it <名称> /bin/bash` | 在运行容器中执行命令 |
| 查看日志 | `container logs <名称>` | 查看容器输出日志 |

---

## 二、用户管理

### 2.1 创建用户并赋予 sudo 权限

默认情况下，容器以 root 用户登录。为了安全性，建议创建普通用户。

```bash
# 1. 创建用户（-m 创建家目录，-s 指定 shell）
useradd -m -s /bin/bash <your_username>

# 2. 设置用户密码
passwd <your_username>

# 3. 将用户添加到 sudo 组
usermod -aG sudo <your_username>

# 4. 切换到该用户
su - <your_username>
```

---

### 2.2 用户切换

**从普通用户切换回 root：**

```bash
# 方法1：退出登录（回到 root）
logout

# 方法2：使用 su 命令（需输入 root 密码）
su - 
```

**从 root 切换到普通用户：**

```bash
su - <your_username>
```

---

### 2.3 配置免密 sudo

避免每次执行 sudo 命令时都输入密码（适用于个人开发环境）。

**⚠️ 安全提示：** 此配置仅适合个人开发环境，生产环境不推荐。

```bash
# 确保处于 root 用户下
echo "<your_username> ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
```

配置完成后，`<your_username>` 用户执行 sudo 命令无需输入密码，永久生效。

---

## 三、实用脚本

### 3.1 容器自动启动脚本

创建一个便捷脚本，实现一键启动并进入容器。

**创建脚本文件：**

```bash
touch start_container.sh
chmod +x start_container.sh
```

**编辑脚本内容：**

```bash
#!/bin/bash

# 配置区域：将 <your_container_name> 替换为实际容器名称
CONTAINER_NAME="<your_container_name>"

echo "正在启动容器 $CONTAINER_NAME..."

# 尝试启动容器（如果已运行则忽略）
container start "$CONTAINER_NAME"

# 检查启动是否成功
if [ $? -eq 0 ]; then
    echo "容器已就绪，正在进入 bash..."
    # 进入容器
    container exec -it "$CONTAINER_NAME" bash
else
    echo "启动失败，请检查容器名称是否正确。"
fi
```

**使用方法：**

```bash
./start_container.sh
```

---

## 四、X11 转发配置

> 📌 完整的 X11 转发配置请参考：[Linux 端 X11 转发配置教程](../tools/ssh/Linux_x11.md)

在容器内配置 X11 转发时，需要特别注意：

1. **安装必要的软件包：**
   ```bash
   apt update
   apt install -y x11-apps xauth
   ```

2. **设置 DISPLAY 环境变量：**
   ```bash
   # 指向 Mac 的 IP 地址
   export DISPLAY=$(ipconfig getifaddr en0):0
   ```

3. **测试 X11 转发：**
   ```bash
   xclock
   ```

---

## 五、故障排查

| 问题现象 | 可能原因 | 解决方案 |
|---------|---------|---------|
| 容器启动失败 | container 服务未运行 | 执行 `container system start` |
| 无法进入容器 | 容器未启动 | 执行 `container start <名称>` |
| sudo 需要密码 | 免密配置未生效 | 检查 `/etc/sudoers` 文件 |
| X11 程序无法显示 | DISPLAY 变量未设置 | 检查 `echo $DISPLAY` |
| 网络连接失败 | 容器网络配置问题 | 检查容器网络模式和防火墙 |
