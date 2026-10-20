# Linux 软件源切换教程

本文参考鱼香 ROS 的换源逻辑，将“识别系统 → 选择对应镜像 → 写入源配置 → 更新索引”整理为手动操作。以 **Ubuntu** 为例，不需要运行 FishROS，也不包含测速环节。

## 1. 确认系统架构与版本

查看软件包架构与系统版本：

```bash
dpkg --print-architecture
cat /etc/os-release
```

### 架构对应关系

| 架构输出 | 含义 | Ubuntu 仓库类型 |
| --- | --- | --- |
| `amd64` | 64 位 x86，即 X64，适用于 Intel 或 AMD 处理器 | `ubuntu` |
| `i386` | 32 位 x86，即常说的 X86 | `ubuntu`，需确认发行版仍支持该架构 |
| `arm64`、`armhf` | ARM 架构 | `ubuntu-ports` |

也可以使用 `uname -m` 查看内核架构：`x86_64` 表示 64 位 x86，`aarch64` 表示 64 位 ARM。选择软件源时以 `dpkg` 显示的软件包架构为准。

### 确认版本代号

查看 `/etc/os-release` 中的 `ID` 与 `VERSION_CODENAME`，确认系统为 Ubuntu，并记下代号。

| Ubuntu 版本 | 代号 |
| --- | --- |
| 22.04 | `jammy` |
| 24.04 | `noble` |
| 26.04 | `resolute` |

> 换源只改变软件下载位置，不要通过修改版本代号来升级系统。Debian 等其他发行版不能直接使用下面的 Ubuntu 配置。

## 2. 选择国内镜像源

选择一个镜像站即可。以下链接指向镜像站及其自身的配置帮助页，复制配置前请确认版本与架构。

| 镜像站 | 站点入口 | 官方使用帮助 |
| --- | --- | --- |
| 中国科学技术大学 | [镜像站](https://mirrors.ustc.edu.cn/) | [Ubuntu 配置](https://mirrors.ustc.edu.cn/help/ubuntu.html) |
| 清华大学 TUNA | [镜像站](https://mirrors.tuna.tsinghua.edu.cn/) | [Ubuntu 配置](https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/) · [Ubuntu Ports 配置](https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu-ports/) |
| 阿里云 | [镜像站](https://mirrors.aliyun.com/) | [Ubuntu 配置](https://developer.aliyun.com/mirror/ubuntu) |

**X64 / X86 的软件仓库地址示例：**

```text
https://mirrors.ustc.edu.cn/ubuntu/
https://mirrors.aliyun.com/ubuntu/
```

**ARM 的软件仓库地址示例：**

```text
https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/
```

> 选择软件仓库 `ubuntu` 或 `ubuntu-ports`，不要使用下载系统 ISO 的 `ubuntu-releases` 地址。

## 3. 确认源文件并备份

Ubuntu 常见以下两种格式，以本机实际配置为准，**只选择对应的一种修改**。

| 格式 | 常见文件位置 | 特征 |
| --- | --- | --- |
| 传统列表 | `/etc/apt/sources.list` | 配置行以 `deb` 开头，Ubuntu 22.04 等系统常见 |
| DEB822 | `/etc/apt/sources.list.d/ubuntu.sources` | 使用 `Types:`、`URIs:` 等字段，Ubuntu 24.04 起默认常见 |

查看本机文件：

```bash
ls -l /etc/apt/sources.list
ls -l /etc/apt/sources.list.d/
```

其中某个文件不存在时，以实际存在并启用的 Ubuntu 系统源文件为准。第三方软件源（如 ROS、Docker）无需删除。

### 传统格式备份

```bash
sudo cp -a /etc/apt/sources.list /etc/apt/sources.list.bak
```

### DEB822 格式备份

```bash
sudo cp -a /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak
```

如果已有需要保留的 `.bak` 文件，请使用其他备份文件名。

## 4. 修改软件源

下面两种格式二选一。示例采用 **Ubuntu 24.04（noble）+ X64 + 中科大镜像**；其他版本或架构请按第 1、2 节替换对应的代号和镜像地址，或直接复制镜像站帮助页生成的配置。

### 方式一：传统 `sources.list` 格式

打开文件：

```bash
sudoedit /etc/apt/sources.list
```

将原有 Ubuntu 系统源条目替换为以下内容；若文件中还有第三方源条目，保留这些条目：

```text
deb https://mirrors.ustc.edu.cn/ubuntu/ noble main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ noble-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ noble-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ noble-security main restricted universe multiverse
```

### 方式二：DEB822 `.sources` 格式

打开文件：

```bash
sudoedit /etc/apt/sources.list.d/ubuntu.sources
```

将原有 Ubuntu 系统源配置替换为：

```text
Types: deb
URIs: https://mirrors.ustc.edu.cn/ubuntu/
Suites: noble noble-updates noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: https://mirrors.ustc.edu.cn/ubuntu/
Suites: noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
```

### 替换时检查这几项

- **版本代号一致：**例如 Ubuntu 22.04 要把示例中的所有 `noble` 换成 `jammy`，包括带 `-updates`、`-backports`、`-security` 的条目。
- **仓库匹配架构：**ARM 系统应使用选定镜像站的 `ubuntu-ports` 地址，不要只改版本代号。
- **避免重复配置：**修改已有的系统源文件，不要在 `sources.list` 和 `ubuntu.sources` 中同时启用同一份源。
- **保留签名校验：**DEB822 配置中的 `Signed-By` 字段无需因更换镜像站而删除。

> 上面与 FishROS 的模板思路一致，将安全更新也指向同一个镜像。若希望保留官方安全更新源，可将 `noble-security` 对应的地址改为 `http://security.ubuntu.com/ubuntu/`；ARM 则使用 `http://ports.ubuntu.com/ubuntu-ports/`。镜像同步可能存在延迟，中科大帮助页建议保留官方安全更新源。

## 5. 更新索引并检查结果

保存文件后执行：

```bash
sudo apt update
```

确认没有下载、签名或重复源错误后，换源即完成，不需要执行系统升级。

| 提示 | 检查内容 |
| --- | --- |
| `404`、没有 Release 文件 | 版本代号、仓库地址及该版本是否仍被镜像站支持 |
| 重复配置警告 | 是否在多个文件中启用了相同源 |
| 签名校验失败 | 配置、系统时间及密钥环；不要直接关闭签名校验 |

## 6. 恢复原配置

如果需要撤销换源，按修改的文件选择一条恢复命令。

**传统格式：**

```bash
sudo cp -a /etc/apt/sources.list.bak /etc/apt/sources.list
```

**DEB822 格式：**

```bash
sudo cp -a /etc/apt/sources.list.d/ubuntu.sources.bak /etc/apt/sources.list.d/ubuntu.sources
```

恢复后重新执行：

```bash
sudo apt update
```

## 参考资料

- [FishROS 系统源配置源码](https://github.com/fishros/install/blob/master/tools/tool_config_system_source.py)：参考其按系统、架构和代号生成源配置的逻辑。
- [中科大 Ubuntu 镜像帮助](https://mirrors.ustc.edu.cn/help/ubuntu.html)：配置格式、仓库地址与安全更新说明。
- [阿里云 Ubuntu 镜像帮助](https://developer.aliyun.com/mirror/ubuntu)：按版本提供的源配置。
- [清华 Ubuntu Ports 镜像帮助](https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu-ports/)：ARM 等架构的源配置。

本教程保留现有配置格式与第三方源，不照搬脚本删除旧配置的操作。镜像配置与参考源码核对日期：2026-10-20。
