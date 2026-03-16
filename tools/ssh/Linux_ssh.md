# SSH 入门指南
## 1. 环境准备
### Linux 端安装 SSH 服务器
如果目标机器是 Linux，通常需要先安装 SSH 服务端：
```bash
sudo apt install openssh-server
```
### 解决连接失败问题
如果 SSH 连接失败（通常是因为目标主机密钥发生变化），可以尝试清除旧的主机密钥记录：
```bash
# 将 IP 替换为实际的目标 IP 地址
ssh-keygen -R 192.168.x.x
```
---
## 2. 配置 SSH 免密登录
### 第一步：检查本地密钥
在本地终端检查是否已有 SSH 密钥对：
```bash
ls -al ~/.ssh
```
*   如果看到 `id_rsa` / `id_rsa.pub` 或 `id_ed25519` / `id_ed25519.pub`，说明已有密钥，可跳过生成步骤。
*   如果没有，请执行下一步生成新密钥。
### 第二步：生成新的 SSH 密钥
推荐使用更安全、性能更好的 `ed25519` 算法：
```bash
# 推荐方式
ssh-keygen -t ed25519 -C "your_email@example.com"
# 或使用传统 RSA 算法
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```
*   按提示操作，可直接按回车使用默认路径和空密码。
### 第三步：上传公钥到服务器
1.  **查看并复制公钥**：
    在本地终端执行，并复制输出的完整内容：
    ```bash
    cat ~/.ssh/id_ed25519.pub
    ```
2.  **登录远程服务器**：
    使用密码登录到您的远程服务器。
3.  **配置服务器端文件**：
    在远程服务器上执行以下命令：
    ```bash
    # 确保 .ssh 目录存在且权限正确
    mkdir -p ~/.ssh
    chmod 700 ~/.ssh
    # 编辑 authorized_keys 文件
    nano ~/.ssh/authorized_keys
    ```
    *   **粘贴公钥**：将本地复制的公钥粘贴到文件末尾（如有多条，需另起一行）。
    *   **保存退出**：在 nano 编辑器中按 `Ctrl+O` 回车保存，然后按 `Ctrl+X` 退出。
4.  **设置文件权限（关键步骤）**：
    权限设置不正确会导致密钥验证失败，请务必执行：
    ```bash
    chmod 600 ~/.ssh/authorized_keys
    ```
### 第四步：验证连接
退出服务器登录：
```bash
exit
```
再次尝试连接，检查是否已实现免密登录：
```bash
ssh username@192.168.x.x
```
---
## 3. （进阶）使用 Config 文件简化登录
如果您需要频繁连接服务器，可以配置本地 `~/.ssh/config` 文件，使用别名快速登录。
### 1. 编辑配置文件
在**本地终端**执行：
```bash
nano ~/.ssh/config
```
### 2. 添加主机配置
写入以下内容（根据实际情况修改）：
```text
Host myserver                # 自定义别名，例如 myserver
    HostName 192.168.1.100   # 服务器 IP 或域名
    User username            # 登录用户名
    Port 22                  # SSH 端口，默认为 22
    IdentityFile ~/.ssh/id_ed25519  # 指定私钥路径
```
*保存并退出（`Ctrl+O` -> `Ctrl+X`）。*
### 3. 快速连接
配置完成后，只需在终端输入别名即可连接：
```bash
ssh myserver
```
