# Linux下Codex安装方法

依次执行：

```bash
# 1. 安装 curl（如果已经安装可以跳过）
sudo apt update
sudo apt install -y curl

# 2. 安装 nvm
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.5/install.sh | bash

# 3. 让当前终端加载 nvm
source ~/.bashrc

# 4. 安装 Node.js 24 LTS
nvm install 24

# 5. 设置 Node.js 24 为默认版本
nvm alias default 24

# 6. 检查 Node.js 和 npm
node -v
npm -v
```

Node.js 官方当前将 **Node.js 24** 列为 LTS 分支；安装 Node.js 后会自带 npm。([Node.js][2])

然后安装 Codex：

```bash
npm install -g @openai/codex
```

检查安装：

```bash
codex --version
```

启动：

```bash
codex
```

使用 nvm 时，通常**不要加 `sudo`**：

```bash
# 正确
npm install -g @openai/codex

# 不推荐
sudo npm install -g @openai/codex
```

OpenAI 的 Codex CLI 就是运行在本地终端中的编码代理。([GitHub][3])

### 一次复制执行版

```bash
sudo apt update && \
sudo apt install -y curl && \
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.5/install.sh | bash && \
source ~/.bashrc && \
nvm install 24 && \
nvm alias default 24 && \
npm install -g @openai/codex && \
node -v && npm -v && codex --version
```

如果执行 `source ~/.bashrc` 后提示：

```text
nvm: command not found
```

可以手动执行：

```bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
```

再运行：

```bash
nvm install 24
npm install -g @openai/codex
```


