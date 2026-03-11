# SSH 入门

如果ssh 失败可以尝试以下命令：
```bash
ssh-keygen -R 192.168.x.x
```
Linux端ssh服务器
```bash
sudo apt install openssh-server
```
# Raspberry Pi SSH 公钥配置成功步骤

> 时间：2026-03-11 21:30  
> 目标：Raspberry Pi @ 192.168.2.203

---

## 环境信息

| 项目 | 值 |
|------|-----|
| 树莓派 IP | 192.168.2.203 |
| 用户名 | pi |
| 密码 | raspberry |
| 系统 | Ubuntu on Raspberry Pi (Linux 5.15.0-1061-raspi aarch64) |
| Mac IP | 192.168.2.141 |

---

## 成功步骤

### 1. 确认 Mac 有 SSH 公钥

```bash
cat ~/.ssh/id_rsa.pub 2>/dev/null || cat ~/.ssh/id_ed25519.pub 2>/dev/null
```

输出：
```
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIJUw1etQ93tMrRljmKYDXqm/kPXSR+OzgHfmmcwX7sLn epsilon5400@gmail.com
```

### 2. 使用 expect 自动配置公钥

创建 expect 脚本：

```bash
cat > /tmp/setup_pi_ssh.exp << 'EOF'
#!/usr/bin/expect -f
set timeout 30
set password "raspberry"
set pubkey "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIJUw1etQ93tMrRljmKYDXqm/kPXSR+OzgHfmmcwX7sLn epsilon5400@gmail.com"

spawn ssh -o StrictHostKeyChecking=no pi@192.168.2.203 "mkdir -p ~/.ssh && echo '$pubkey' >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys && echo 'SSH_KEY_ADDED_SUCCESS'"

expect {
    "password:" {
        send "$password\r"
        exp_continue
    }
    "SSH_KEY_ADDED_SUCCESS" {
        puts "\nSUCCESS: Public key configured!"
        exit 0
    }
    timeout {
        puts "\nFAILED: Timeout"
        exit 1
    }
    eof {
        puts "\nConnection closed"
        exit 1
    }
}
EOF

chmod +x /tmp/setup_pi_ssh.exp
/tmp/setup_pi_ssh.exp
```

### 3. 测试无密码登录

```bash
ssh -o BatchMode=yes -o ConnectTimeout=10 pi@192.168.2.203 "echo 'SUCCESS' && hostname && uname -a"
```

输出：
```
PASSWORDLESS_LOGIN_SUCCESS
raspberrypi
Linux raspberrypi 5.15.0-1061-raspi #64-Ubuntu SMP PREEMPT Wed Aug 7 14:41:40 UTC 2024 aarch64 aarch64 aarch64 GNU/Linux
```

---

## 关键点

1. **使用 expect 自动化**：避免手动输入密码，适合脚本化部署
2. **StrictHostKeyChecking=no**：首次连接跳过主机密钥确认
3. **chmod 700 ~/.ssh**：目录权限必须是 700
4. **chmod 600 authorized_keys**：公钥文件权限必须是 600
5. **BatchMode=yes**：测试时禁用密码登录，确保只用公钥

---

## 踩过的坑

### ❌ 错误 1：用户名错误
- 第一次烧录时用户名设为 `raspberry`，但 SSH 登录失败
- 解决：重刷系统，使用默认用户名 `pi`

### ❌ 错误 2：烧录时公钥未生效
- 烧录软件中配置了公钥，但实际未生效
- 解决：手动通过 expect 脚本配置

### ❌ 错误 3：权限问题
- `~/.ssh` 目录权限不对会导致公钥认证失败
- 解决：确保目录 700，文件 600

---

## 后续步骤

配置公钥成功后，可以继续：
1. 检查/安装 Node.js
2. 安装 OpenClaw: `npm install -g openclaw`
3. 运行 Node 连接到 Mac Gateway

---

_此文档记录成功步骤，避免重复踩坑_
