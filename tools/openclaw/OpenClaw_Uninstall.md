这是一份专为 Linux 和 macOS 用户设计的 OpenClaw 彻底卸载指南。

本指南基于官方文档及社区安全最佳实践整理，旨在帮助你不仅删除软件，更要清除所有配置残留、后台服务及潜在的安全隐患（特别是 API 密钥和 OAuth 授权）。

🦞 OpenClaw 彻底卸载指南 (Linux & macOS)

⚠️ 重要提示：
顺序至关重要：请先执行“官方卸载命令”，再手动删除文件和 npm 包。如果先删了 npm 包，openclaw 命令将失效，导致无法运行官方清理脚本。
安全第一：卸载本地文件不会自动撤销云端的 OAuth 授权或使 API 密钥失效。请务必执行“第五步”。

📋 卸载清单概览
步骤   操作内容   关键命令/动作
1   备份配置 (可选)   cp -r ~/.openclaw ~/backup...

2   停止服务   systemctl (Linux) / launchctl (macOS)

3   官方卸载   openclaw uninstall --all

4   手动清理   删除残留目录、npm 包、Docker

5   撤销授权   手动去 Google/GitHub 等平台撤销

6   验证   检查进程、端口、文件

🛑 第一步：备份配置 (可选但推荐)

如果你未来可能重装，或需要保留某些工作区文件，请先备份。

创建备份目录
BACKUP_DIR=~/openclaw-backup-(date +%Y%m%d-%H%M%S)
mkdir -p "BACKUP_DIR"

备份主配置 (包含 API Key, 记忆, 工作区)
if [ -d ~/.openclaw ]; then
    cp -r ~/.openclaw "BACKUP_DIR/"
    echo "✅ 已备份 ~/.openclaw 到 BACKUP_DIR"
fi

备份历史版本残留 (如果有)
for dir in ~/.clawdbot ~/.moltbot ~/.molthub; do
    if [ -d "dir" ]; then
        cp -r "dir" "BACKUP_DIR/"
        echo "✅ 已备份 dir 到 BACKUP_DIR"
    fi
done

🛑 第二步：停止后台服务 (Gateway)

OpenClaw 会运行一个名为 Gateway 的后台守护进程（监听端口 18789），必须先停止它。

🐧 Linux 用户
停止并禁用 systemd 用户服务
systemctl --user stop openclaw-gateway.service
systemctl --user disable openclaw-gateway.service

删除服务文件
rm -f ~/.config/systemd/user/openclaw-gateway.service
systemctl --user daemon-reload

强制杀掉残留进程
pkill -f openclaw || true

🍎 macOS 用户
停止并卸载 LaunchAgent (尝试多种可能的服务名)
launchctl bootout gui/UID/ai.openclaw.gateway 2>/dev/null
launchctl bootout gui/UID/bot.molt.gateway 2>/dev/null
launchctl bootout gui/UID/com.openclaw.gateway 2>/dev/null

删除 plist 文件
rm -f ~/Library/LaunchAgents/ai.openclaw.gateway.plist
rm -f ~/Library/LaunchAgents/bot.molt.gateway.plist
rm -f ~/Library/LaunchAgents/com.openclaw.gateway.plist

强制杀掉残留进程
pkill -f openclaw || true

🛑 第三步：执行官方卸载命令

这是最关键的一步。只要 openclaw 命令还能用，请优先使用官方脚本。

执行官方一键卸载 (自动停止服务、删除配置、清理环境)
openclaw uninstall --all --yes --non-interactive

💡 如果提示 command not found：
说明你可能已经误删了 npm 包。此时可以用 npx 临时调用：
npx -y openclaw uninstall --all --yes --non-interactive
🛑 第四步：手动清理残留 (斩草除根)

官方命令可能不会清理历史版本（Clawdbot/Moltbot）的目录，也不会删除全局 npm 包（视版本而定），需手动执行。

删除配置目录
删除当前版本配置
rm -rf ~/.openclaw

⚠️ 重要：删除历史版本残留 (很多人忽略这一步)
rm -rf ~/.clawdbot
rm -rf ~/.moltbot
rm -rf ~/.molthub

macOS 特有清理
rm -rf /Applications/OpenClaw.app
rm -rf /tmp/openclaw

卸载 CLI 工具
根据你的安装方式选择执行：
NPM 安装
npm uninstall -g openclaw
npm uninstall -g @qingchencloud/openclaw-zh  # 如果装过中文汉化版

PNPM 安装
pnpm remove -g openclaw

BUN 安装
bun remove -g openclaw

清理 Docker (如果用过 Docker 部署)
停止并删除容器
docker stop openclaw 2>/dev/null && docker rm openclaw 2>/dev/null

删除数据卷
docker volume rm openclaw-data 2>/dev/null

删除镜像
docker rmi openclaw/openclaw:latest 2>/dev/null

如果使用 docker-compose
cd 到你的 compose 目录
docker compose down --volumes 2>/dev/null

清理 Shell 配置
检查 ~/.bashrc, ~/.zshrc, ~/.bash_profile，删除包含 openclaw 的行（通常是 export PATH 或环境变量）。
示例：编辑配置文件
nano ~/.zshrc  # 或 vim ~/.bashrc
找到类似 export OPENCLAW_HOME=... 的行并删除，然后 source 生效
source ~/.zshrc

🛑 第五步：撤销云端授权 (❗必须手动操作)

本地删除文件 ≠ 撤销权限！ OpenClaw 持有的 OAuth 令牌和 API Key 仍然有效。为了防止泄露，请务必执行以下操作：

轮换 API 密钥
如果你曾在 ~/.openclaw/openclaw.json 中配置过以下密钥，请立即去对应官网删除旧密钥并生成新的：
OpenAI / Claude / DeepSeek / Google AI
AWS / Azure / 阿里云 Access Key
任何第三方服务 Token

撤销 OAuth 授权
前往以下平台，找到 OpenClaw (或 Clawdbot/Moltbot) 并点击 移除/撤销 (Revoke)：
平台   撤销路径
Google   myaccount.google.com/permissions

GitHub   Settings → Applications → Authorized OAuth Apps

Slack   slack.com/apps/manage → Your Apps

Discord   User Settings → Authorized Apps

Notion   Settings & Members → Connections

Microsoft   account.live.com/consent/Manage

🛑 第六步：验证卸载结果

运行以下脚本，确保系统已干净。

echo "🔍 开始验证..."

检查命令是否存在 (应无输出或提示 not found)
if command -v openclaw &> /dev/null; then
    echo "❌ 失败：openclaw 命令仍存在"
else
    echo "✅ 通过：openclaw 命令已移除"
fi

检查目录残留 (应全部提示 No such file)
RESIDUE_FOUND=false
for dir in ~/.openclaw ~/.clawdbot ~/.moltbot ~/.molthub; do
    if [ -d "dir" ]; then
        echo "❌ 失败：发现残留目录 dir"
        RESIDUE_FOUND=true
    fi
done
if [ "RESIDUE_FOUND" = false ]; then
    echo "✅ 通过：所有配置目录已清空"
fi

检查进程 (应只有 grep 自身)
PROC_COUNT=(ps aux | grep -i openclaw | grep -v grep | wc -l)
if [ "PROC_COUNT" -gt 0 ]; then
    echo "❌ 失败：发现残留进程"
    ps aux | grep -i openclaw | grep -v grep
else
    echo "✅ 通过：无残留进程"
fi

检查端口 18789 (应无输出)
if command -v lsof &> /dev/null; then
    PORT_CHECK=(lsof -i :18789)
    if [ -n "$PORT_CHECK" ]; then
        echo "❌ 失败：端口 18789 仍被占用"
    else
        echo "✅ 通过：端口 18789 已释放"
    fi
fi

echo "🎉 验证完成！"

❓ 常见问题 (FAQ)

Q: 我直接 rm -rf 了目录，现在怎么办？
A: 如果 openclaw 命令还在，赶紧运行 openclaw uninstall --all 来清理服务和注册表。如果命令也没了，请按“第四步”手动清理服务文件（systemd/launchctl），否则开机可能报错。

Q: 为什么卸载后还要撤销授权？
A: OpenClaw 使用的是长期有效的 OAuth Token。即使你删了软件，黑客如果获取了你的 Token（例如通过之前的内存泄露或日志），依然可以用它访问你的 Google 邮箱或 GitHub 代码库。轮换密钥是必须的。

Q: 我想重新安装，怎么恢复配置？
A: 如果你在第一步做了备份，重装后可以将备份文件夹复制回 ~/.openclaw。或者使用社区工具 openclaw-uninstaller 的快照功能进行恢复。
祝你卸载顺利，系统清爽！🌟
