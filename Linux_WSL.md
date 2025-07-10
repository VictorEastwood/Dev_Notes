# WSL入门

### WSL安装
1. 打开PowerShell（以管理员身份运行）
2. 运行以下命令启用WSL：
   ```powershell
   wsl --install
   ```
3. 查看允许安装的Linux发行版：
   ```powershell
   wsl --list --online
   ```
4. 选择一个Linux发行版进行安装，例如Ubuntu 22.04：
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

### wsl卸载
1. 查看wsl状态
    ```powershell
    wsl --list --verbose
    ```

2. 关闭正在运行的wsl
    ```powershell
    wsl --shutdown
    ```

3. 导出wsl备份
    ```powershell
    wsl --export Ubuntu-22.04 D:\Ubuntu_WSL\Ubuntu.tar
    ```

4. 卸载wsl
    ```powershell
    wsl --unregister Ubuntu-22.04
    ```
5. 导入wsl
    ```powershell
    wsl --import Ubuntu-22.04 D:\Ubuntu_WSL D:\Ubuntu_WSL\Ubuntu.tar
    ```
