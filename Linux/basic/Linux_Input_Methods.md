# Linux 中文输入法与字体配置

本文整理 Ubuntu 桌面环境下的中文输入法配置方法。搜狗、谷歌拼音和 IBus 为不同方案，按需选择其中一种。

## 1. 搜狗输入法（Fcitx）

### 1.1 更新软件包索引

```bash
sudo apt update
```

### 1.2 安装 Fcitx 输入法框架

```bash
sudo apt install fcitx
```

打开“语言支持”（Language Support），将“键盘输入法系统”设置为 `fcitx`。

### 1.3 设置开机自启动

```bash
sudo cp /usr/share/applications/fcitx.desktop /etc/xdg/autostart/
```

### 1.4 按需卸载 IBus

原笔记使用以下命令卸载 IBus。仅在确认切换到 Fcitx、且不再需要 IBus 时执行；确认 APT 列出的待移除软件后再继续。

```bash
sudo apt purge ibus
```

### 1.5 下载并安装搜狗输入法

访问 [搜狗输入法 Linux 官网](https://shurufa.sogou.com/linux)，选择适合当前系统版本和架构的安装包。具体兼容要求参见 [官方安装指导](https://shurufa.sogou.com/linux/guide)。

进入下载目录，将 `sogoupinyin.deb` 替换为实际下载的文件名：

```bash
cd ~/Downloads
sudo dpkg -i ./sogoupinyin.deb
```

### 1.6 安装依赖

```bash
sudo apt install libqt5qml5 libqt5quick5 libqt5quickwidgets5 qml-module-qtquick2
sudo apt install libgsettings-qt1
```

### 1.7 重启并检查

重启电脑后，在 Fcitx 配置中检查是否已添加搜狗输入法。

## 2. 谷歌拼音（Fcitx）

在使用 Fcitx 的环境中安装谷歌拼音：

```bash
sudo apt install fcitx-googlepinyin
```

安装后，在 Fcitx 配置中添加谷歌拼音。

## 3. 系统默认输入法（IBus）

1. 打开“语言支持”（Language Support），安装中文语言包。
2. 如果希望使用中文系统界面，将中文拖到语言列表最上方。
3. 在“语言支持”中，将“键盘输入法系统”设置为 `IBus`，按需应用到整个系统。
4. 重启电脑，并在系统设置的输入源中添加中文输入法。

> 如果希望保留终端中常用的英文目录名，重新登录时选择保留 `Desktop`、`Downloads` 等原目录名称。

## 4. 调整中文字体字形

如果英文系统中的中文显示为日文字形，可以检查语言字体优先级。原笔记使用的配置文件如下；不同系统版本的文件可能不同，请先确认该文件存在。

先备份，再打开编辑器：

```bash
sudo cp -a /etc/fonts/conf.avail/64-language-selector-prefer.conf /etc/fonts/conf.avail/64-language-selector-prefer.conf.bak
sudoedit /etc/fonts/conf.avail/64-language-selector-prefer.conf
```

在相应的字体优先级列表中，将简体中文字体（名称中含 `SC`，即 Simplified Chinese）放到日文字体之前，保持 XML 结构完整。
