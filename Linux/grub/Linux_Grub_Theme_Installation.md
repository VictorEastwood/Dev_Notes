# GRUB 引导主题安装

本文以 Ubuntu 的 GRUB 环境为例，按“下载 → 解压 → 移动 → 安装”的顺序整理。以下示例将主题压缩包命名为 `your_theme.tar.gz`，解压后的目录命名为 `your_theme`，使用时替换为实际名称。

## 1. 下载主题

在 [GNOME-Look GRUB 主题分类](https://www.gnome-look.org/browse?cat=109&ord=latest) 中选择主题，将压缩包保存到 `~/Downloads`。

## 2. 解压主题

进入下载目录：

```bash
cd ~/Downloads
```

根据压缩包格式，选择一条解压命令执行。

**`.tar.gz` 格式：**

```bash
tar -zxvf your_theme.tar.gz
```

**`.zip` 格式：**

```bash
unzip your_theme.zip
```

查看解压结果，确认实际主题目录。若压缩包包含多层目录，应先查看随附的 README，找到安装脚本所在位置。

```bash
ls
ls your_theme
```

## 3. 移动主题目录

创建主题存放目录，再将解压后的目录移动过去：

```bash
sudo mkdir -p /boot/grub/themes
sudo mv your_theme /boot/grub/themes/
```

> 移动的是解压后的目录，不是压缩包。这里保持普通用户终端，仅在需要权限的命令前使用 `sudo`，避免切换到 root 后 `~` 指向 `/root`。

## 4. 安装主题

进入主题目录，先阅读主题说明与安装脚本：

```bash
cd /boot/grub/themes/your_theme
less README.md
less install.sh
```

`README.md` 为示例文件名，以主题实际附带的说明文件为准。确认脚本适用于当前系统后，按主题说明执行安装；对于附带 `install.sh` 且支持此调用方式的主题：

```bash
sudo bash ./install.sh
```

> 各主题的脚本位置、参数和目标路径可能不同。若没有 `install.sh`，按主题作者的说明安装。

## 5. 更新配置并重启

在 Ubuntu 上更新 GRUB 配置，确认命令成功后重启：

```bash
sudo update-grub
```

```bash
sudo reboot
```
