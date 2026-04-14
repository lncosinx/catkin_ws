在 WSL 2 中原生是无法直接访问主机的 USB 设备的（除了鼠标、键盘等基础输入设备）。为了让 WSL 2 能够识别并使用 USB 设备（例如串口设备、摄像头、调试器等），微软官方推荐使用开源工具 **usbipd-win**。

以下是完整的配置和使用步骤：

### **第一步：在 Windows 上安装 usbipd-win**

1. 打开 Windows 的 **PowerShell**。  
2. 运行以下 winget 命令来安装 usbipd-win：  
   ```powerShell  
   winget install \-\-interactive \-\-exact dorssel.usbipd\-win
   ```

   *(如果不想用命令行，也可以去 GitHub 的 [usbipd-win 发布页面](https://github.com/dorssel/usbipd-win/releases) 下载 .msi 安装包手动安装。)*  
3. **安装完成后，请重启你的计算机**，或者重新启动终端，以确保环境变量和后台服务生效。

### **第二步：在 WSL 中安装 USB 工具**

为了让 Linux 内核能够处理转发过来的 USB 数据，需要在 WSL 中安装相应的硬件工具。

1. 打开你的 WSL 终端（以最常见的 Ubuntu 为例）。  
2. 更新软件源并安装必要的包：  
```bash  
sudo apt update  
sudo apt install linux-tools-virtual hwdata
```

### **第三步：将 USB 设备连接到 WSL**

接下来需要将物理插入 Windows 的 USB 设备“桥接”到 WSL 中。

1. **以管理员身份**打开 Windows PowerShell。  
2. 列出当前连接到 Windows 的所有 USB 设备：  
```powerShell  
usbipd list
```

   你会看到一个设备列表，找到你想要连接到 WSL 的那个设备，并记下它的 **BUSID**（例如 2-1）。  
3. 共享该设备（绑定）：  
```powerShell  
usbipd bind \-\-busid \<BUSID\>
```

   *注意：这个步骤通常只需要对该设备执行一次，以后即使拔插也无需重新绑定。*  
4. 将设备附加到 WSL：  
```powerShell  
   usbipd attach \-\-wsl \-\-busid \<BUSID\>
```

   *注意：执行此命令时，确保你的 WSL 正在后台运行。*

### **第四步：在 WSL 中验证**

回到你的 WSL 终端，输入以下命令查看已连接的 USB 设备：

```bash
lsusb
```
如果一切顺利，你应该能在输出的列表中看到你刚刚桥接进来的 USB 设备。此时你就可以像在原生 Linux 中一样，通过 /dev/ttyUSB0、/dev/video0 或其他相应的挂载点来使用它了。

### **第五步：断开连接**

当你用完设备，或者想让 Windows 重新接管该 USB 设备时，可以在 Windows PowerShell 中运行：

```powerShell
usbipd detach \-\-busid \<BUSID\>
```

*(或者直接拔下该 USB 设备也会自动断开连接。)*