# 我的开发笔记

记录日常开发中用到的环境配置、工具使用、命令速查和代码模板，按 **Linux** 与 **macOS** 两个平台整理，方便搭建环境和回查问题。

## 目录结构

```text
Dev_Notes/
├── Linux/
│   ├── basic/     # 基础环境、软件源、输入法与常用命令
│   ├── grub/      # GRUB 引导主题安装
│   ├── git/       # Git 使用与入门练习
│   ├── ssh/       # 远程连接、文件传输与 X11 转发
│   ├── vim/       # Vim 使用指南与入门练习
│   ├── python/    # Conda 环境管理
│   ├── ros2/      # ROS 2 安装、命令与代码模板
│   ├── serial/    # 串口驱动配置与使用技巧
│   ├── kdl/       # KDL 运动学库
│   ├── opencv/    # OpenCV 视觉库
│   ├── wsl/       # WSL 与 USB 设备连接
│   └── agent/     # AI 开发工具
├── Mac/          # macOS 命令、快捷键、容器与虚拟机
└── README.md
```

## Linux 笔记

### 基础环境

- [基础工作环境配置](Linux/basic/Linux_setup.md)：系统安装、常用软件与系统设置。
- [基础开发环境搭建](Linux/basic/Linux_development.md)：常用工具、C++ 与 Python 开发环境。
- [常用命令速查](Linux/basic/Linux_cmd.md)：日常终端操作参考。

- [软件源切换教程](Linux/basic/Linux_Source.md)：架构判断、国内镜像站、手动换源与配置恢复。
- [中文输入法与字体配置](Linux/basic/Linux_Input_Methods.md)：搜狗、谷歌拼音、IBus 与字体字形。

### 桌面与引导主题

- [GRUB 引导主题安装](Linux/grub/Linux_Grub_Theme_Installation.md)：下载、解压、移动主题与安装。

### Git 与 Vim

| 主题 | 使用参考 | 入门练习 |
| --- | --- | --- |
| Git | [Git 常用操作](Linux/git/Linux_git.md) | [Git 零基础教程](Linux/git/Linux_git_tutorial.md) |
| Vim | [Vim 使用指南](Linux/vim/Linux_vim.md) | [Vim 零基础教程](Linux/vim/Linux_vim_tutorial.md) |

### 远程连接

- [SSH 入门指南](Linux/ssh/Linux_ssh.md)
- [SCP 文件传输](Linux/ssh/Linux_scp.md)
- [X11 图形界面转发](Linux/ssh/Linux_x11.md)

### Python 与 ROS 2

- [Conda 环境管理](Linux/python/Linux_conda.md)
- [ROS 2 安装与配置](Linux/ros2/Linux_ros2_installation.md)
- [ROS 2 常用命令](Linux/ros2/Linux_ros2_cmd.md)
- [ROS 2 代码模板](Linux/ros2/Linux_ros2_template.md)

### 串口通信

- [串口驱动安装与配置](Linux/serial/Linux_Serial_Driver_Installation.md)：`brltty` 冲突处理、CH34X 驱动安装与串口访问权限配置。
- [串口使用技巧](Linux/serial/Linux_Serial_Tools.md)：设备识别、占用排查、回环测试与调试工具。

### 开发库

- [KDL 安装指南](Linux/kdl/Linux_kdl_installation.md.md)
- [OpenCV 与 opencv-contrib 安装配置](Linux/opencv/Linux_opencv_installation.md)

### WSL 与 AI 工具

- [WSL 使用教程](Linux/wsl/Linux_wsl.md)：子系统管理、USB 设备连接与问题排查。
- [Codex 安装笔记](Linux/agent/Codex.md)

## macOS 笔记

- [macOS 常用命令速查](Mac/Mac_cmd.md)
- [Mac 键盘快捷键](Mac/Mac_shortcuts.md)
- [Apple Container 配置指南](Mac/Apple_container.md)：容器管理、用户管理、实用脚本与 X11 转发。
- [Mac 安装 Linux 虚拟机](Mac/Mac_Linux_VM.md)：ARM64 镜像与虚拟机基础环境配置。

## 阅读方式

可以直接点击上方链接阅读，也可以克隆到本地，用 Markdown 编辑器浏览：

```bash
git clone https://github.com/VictorEastwood/Dev_Notes.git
cd Dev_Notes
```

初次搭建 Linux 开发环境时，可依次阅读“基础工作环境配置”和“基础开发环境搭建”，再按需查阅各主题。日常使用时，可直接打开命令速查、工具指南或代码模板。

## 笔记维护约定

- 按平台和主题归档文档，新增或移动文件后同步更新本页导航。
- 安装与配置教程、日常使用技巧分开整理，相关文档之间添加链接。
- 使用清晰的标题层级，为命令和输出标注代码块语言。
- 涉及具体系统版本、硬件型号或环境条件时，在对应文档中说明。

发现错误或有补充建议，欢迎提交 [Issue](https://github.com/VictorEastwood/Dev_Notes/issues) 或 Pull Request。
