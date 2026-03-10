这是一个非常实用的 **SCP (Secure Copy)** 命令速查表，涵盖了从基础传输到高级限速、端口指定等常用场景。
### 1. 核心语法结构
记住这个公式即可：
`scp [选项] 源地址 目标地址`
*   **本地路径**：直接写路径 (如 `/data/file.txt` 或 `.`)
*   **远程路径**：格式为 `用户名@IP:路径` (如 `root@192.168.1.1:/home/data`)
---
### 2. 常用场景指令
#### A. 下载
*把远程文件/文件夹拉取到本地*
| 场景 | 命令示例 |
| :--- | :--- |
| **下载单个文件** | `scp user@host:/remote/file.txt /local/dir/` |
| **下载整个文件夹** | `scp -r user@host:/remote/folder /local/dir/` |
| **下载并重命名** | `scp user@host:/remote/file.txt /local/dir/new_name.txt` |
#### B. 上传
*把本地文件/文件夹推送到远程*
| 场景 | 命令示例 |
| :--- | :--- |
| **上传单个文件** | `scp /local/file.txt user@host:/remote/dir/` |
| **上传整个文件夹** | `scp -r /local/folder user@host:/remote/dir/` |
| **上传并重命名** | `scp /local/file.txt user@host:/remote/dir/new_name.txt` |
#### C. 服务器对传
*两台远程服务器之间传输，数据不经过本地*
| 场景 | 命令示例 |
| :--- | :--- |
| **远程A传到远程B** | `scp userA@hostA:/file.txt userB@hostB:/dir/` |
---
### 3. 常用功能参数
这些参数可以组合使用，例如 `scp -P 2222 -rC ...`。
| 参数 | 全称 | 功能说明 | 示例 |
| :--- | :--- | :--- | :--- |
| **`-r`** | Recursive | **递归复制**（复制文件夹时必须加） | `scp -r folder user@host:/path` |
| **`-P`** | Port | 指定远程 **端口号** (注意是大写P) | `scp -P 2222 file user@host:/path` |
| **`-p`** | Preserve | **保留文件原属性** (小写p，保留修改时间、权限等) | `scp -p file user@host:/path` |
| **`-C`** | Compress | 传输时**压缩**数据 (适于网络慢、传文本日志) | `scp -C big.log user@host:/path` |
| **`-i`** | Identity | 指定**密钥文件**登录 (不用输密码) | `scp -i ~/.ssh/key.pem file user@host:/path` |
| **`-v`** | Verbose | 显示详细连接过程 (排错用) | `scp -v file user@host:/path` |
| **`-q`** | Quiet | 静默模式 (不显示进度条和警告) | `scp -q file user@host:/path` |
| **`-l`** | Limit | **限制传输速度** (单位：Kbit/s) | `scp -l 800 file user@host:/path` (约100KB/s) |
---
### 4. 进阶组合示例
**场景 1：指定端口 + 使用密钥 + 上传文件夹**
(假设 SSH 端口为 2022，使用 `my_key.pem` 密钥)
```bash
scp -P 2022 -i ~/keys/my_key.pem -r /local/data user@host:/remote/backup
```
**场景 2：限速下载 (防止占满带宽)**
(限制速度约为 1MB/s，即 8000 Kbit/s)
```bash
scp -l 8000 user@host:/remote/big_file.iso /local/
```
**场景 3：保留文件属性 + 压缩传输**
(适合备份网站代码，保留原有时间戳且压缩传输)
```bash
scp -p -C -r /var/www/html user@host:/backup/
```
### 5. 注意事项
1.  **端口参数大小写**：这是最容易犯错的地方。指定端口用大写 `-P`，保留属性用小写 `-p`。
2.  **覆盖不提示**：SCP 如果目标文件已存在，会直接覆盖且**不会**询问。如果要对比差异或增量同步，请使用 `rsync`。
3.  **软链接**：默认情况下 SCP 不会复制软链接指向的真实文件，只会复制链接本身。如果需要复制链接指向的实际内容，通常建议使用 `rsync` 或打包后再传。
