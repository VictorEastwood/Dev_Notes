# 🦞 OpenClaw 网关配置与重启速查手册
## 📋 核心流程图
```mermaid
flowchart LR
    A[检查当前状态] --> B[修改配置参数]
    B --> C[安装/重载服务]
    C --> D[验证最终状态]
    
    subgraph B [配置修改]
        B1[设置绑定地址]
        B2[设置端口号]
    end
    
    subgraph C [服务管理]
        C1[安装服务]
        C2[启动/重启服务]
    end
```
## 🔧 步骤一：检查当前状态
**命令**：
```bash
openclaw gateway status
```
**关注点**：
- `Service`: 是否显示 `not loaded` 或 `not installed`
- `Gateway`: 当前 `bind` 和 `port` 值
- `Runtime`: 是否 `running`
---
## ⚙️ 步骤二：修改配置参数
### 1. 修改绑定地址（从 loopback 改为 lan）
```bash
openclaw config set gateway.bind "lan"
```
**作用**：允许局域网设备访问，监听地址从 `127.0.0.1` 变为 `0.0.0.0`。
### 2. 修改端口号（从 18789 改为 xxxxx）
```bash
openclaw config set gateway.port xxxxx
```
**作用**：更改网关监听端口。
---
## 🚀 步骤三：安装与启动服务
### 一键安装并启动（推荐）
```bash
openclaw gateway install && openclaw gateway start
```
### 分步操作
1. **安装服务**：
   ```bash
   openclaw gateway install
   ```
   - 创建 macOS LaunchAgent 服务文件
   - 加载服务到系统
2. **启动/重启服务**：
   ```bash
   openclaw gateway start
   # 或
   openclaw gateway restart
   ```
---
## ✅ 步骤四：验证最终状态
### 1. 检查服务状态
```bash
openclaw gateway status
```
### 2. 预期输出关键指标
| 检查项 | 预期结果 | 说明 |
|--------|----------|------|
| **服务状态** | `LaunchAgent (loaded)` | 服务已正确加载 |
| **运行时** | `running (pid XXXXX)` | 进程正在运行 |
| **绑定地址** | `bind=lan (0.0.0.0)` | 监听所有网络接口 |
| **端口** | `port=20296` | 新端口已生效 |
| **RPC探测** | `ok` | 内部通信正常 |
| **访问地址** | `Dashboard: http://192.168.2.141:xxxxx/` | 可访问的网址 |
### 3. 验证端口监听
```bash
netstat -tuln | grep xxxxx
```
应显示：`tcp4  0  0  *.xxxxx   *.*   LISTEN`
---
## 📚 常用命令速查表
| 操作 | 命令 |
|------|------|
| **检查状态** | `openclaw gateway status` |
| **修改绑定地址** | `openclaw config set gateway.bind "lan"` |
| **修改端口** | `openclaw config set gateway.port 20296` |
| **安装服务** | `openclaw gateway install` |
| **启动服务** | `openclaw gateway start` |
| **重启服务** | `openclaw gateway restart` |
| **停止服务** | `openclaw gateway stop` |
| **卸载服务** | `openclaw gateway uninstall` |
| **查看日志** | `openclaw logs` |
| **系统诊断** | `openclaw doctor` |
---
## 🚨 常见问题排查
### 问题1：服务未加载
**症状**：`Gateway service not loaded`
**解决**：
```bash
openclaw gateway install
openclaw gateway start
```
### 问题2：端口被占用
**症状**：启动失败，日志显示端口错误
**解决**：
```bash
# 检查端口占用
lsof -i :xxxxx
# 终止占用进程或更换端口
openclaw config set gateway.port 新端口号
openclaw gateway restart
```
### 问题3：配置未生效
**症状**：状态显示旧配置
**解决**：
```bash
# 重启服务
openclaw gateway restart
# 如无效，重新安装服务
openclaw gateway uninstall
openclaw gateway install
openclaw gateway start
```
---
## 📁 关键文件位置
| 文件类型 | 路径 |
|----------|------|
| **主配置文件** | `~/.openclaw/openclaw.json` |
| **配置备份** | `~/.openclaw/openclaw.json.bak` |
| **服务文件** | `~/Library/LaunchAgents/ai.openclaw.gateway.plist` |
| **日志文件** | `/tmp/openclaw/openclaw-YYYY-MM-DD.log` |
| **服务日志** | `~/.openclaw/logs/gateway.log` |
---
## 🎯 快速操作清单
- [ ] 执行 `openclaw gateway status` 检查当前状态
- [ ] 执行 `openclaw config set gateway.bind "lan"` 修改绑定地址
- [ ] 执行 `openclaw config set gateway.port xxxxx` 修改端口
- [ ] 执行 `openclaw gateway install && openclaw gateway start` 安装并启动
- [ ] 执行 `openclaw gateway status` 验证最终状态
- [ ] 浏览器访问 `http://<您的IP>:xxxxx/` 确认可访问
---
**备注**：
1. 配置修改后必须重启服务才能生效
2. macOS 系统需要使用 LaunchAgent 管理服务
3. 修改为 `lan` 模式后，局域网设备可通过 IP 地址访问
4. 建议定期检查 `openclaw doctor` 确保系统健康
