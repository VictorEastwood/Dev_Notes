# Linux 基础开发环境搭建

## 1.安装系统常用工具
```bash
sudo apt install -y wget curl git tree vim terminator minicom cutecom
```
## 2.C++开发环境搭建
基础安装c++
```bash
sudo apt install -y build-essential cmake cmake-qt-gui cmake-curses-gui 
```
检测是否安装成功：
```bash
git --version
g++ --version
cmake --version
ccmake --version
```
## 3.Python开发环境搭建
基础安装python
```bash
sudo apt -y install python3 python3-pip python3-venv python-is-python3
```
检测是否安装成功：
```bash
python3 --version
pip3 --version
```

git config --global user.email "epsilon5400@gmail.com"
git config --global user.name "shidong_wu"
