Linux内核
=========

对于内核开发者和用户，有多个指南可供参考。这些指南可以呈现为多种格式，如HTML和PDF。请先阅读Documentation/admin-guide/README.rst。

为了构建文档，请使用``make htmldocs``或``make pdfdocs``。格式化的文档也可以在线阅读：

    https://www.kernel.org/doc/html/latest/

在Documentation/子目录中有各种文本文件，其中一些使用Restructured Text标记符号。
有关每个文件包含的内容列表，请参阅Documentation/00-INDEX。

请阅读Documentation/process/changes.rst文件，因为它包含构建和运行内核的要求，以及升级内核可能导致的问题信息。

## RK3568系列板卡编译指南

> 编译教程参考: https://doc.embedfire.com/linux/rk356x/driver/zh/latest/linux_driver/base_exper_env.html#

### 编译环境准备

#### 获取编译工具链
```bash
# 安装相关库和工具搭建编译环境，执行以下命令：
sudo apt update
sudo apt install gcc make  git  bc libssl-dev liblz4-tool device-tree-compiler bison flex u-boot-tools gcc-aarch64-linux-gnu

# 查看编译工具链，如果COLLECT_LTO_WRAPPER变量为指定的路径，即配置成功
aarch64-linux-gnu-gcc -v
```

#### 获取源码
```bash
git clone https://github.com/Long2025/lubancat-kernel.git

# 下载完成后，进入到所在文件夹，然后修改文件夹名字
mv lubancat-kernrl kernel
```

### 编译内核

#### 在PC上交叉编译内核（建议）
进入内核源码根目录，根据具体的板卡设置配置文件。
```bash
cd kernel
```

RK3568系列板卡用户执行以下命令编译内核源码：

```bash
# 清除之前生成的所有文件和配置
make mrproper

# 加载lubancat2_defconfig配置文件，rk3568系列均是该配置文件
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- lubancat2_defconfig

# 编译内核，指定平台，指定交叉编译工具，使用8线程进行编译，线程可根据电脑性能自行确定
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j8
```

#### 在板卡上本地编译内核
按照前面小节搭建编译环境并下载源码之后，进入内核源码根目录，根据具体的板卡设置配置文件。

RK3568系列板卡用户执行以下命令编译内核源码：

```bash
# 清除之前生成的所有文件和配置
make mrproper

# 加载lubancat2_defconfig配置文件，rk3568系列均是该配置文件
make lubancat2_defconfig

# 编译内核，使用4线程进行编译
make -j4
```

### 常见问题解决

#### Python路径问题
如果在执行 `sudo make mrproper` 时报错 "/usr/bin/env:'python': No such file or directory"，请执行以下命令指向python路径：

```bash
sudo ln -s /usr/bin/python3 /usr/bin/python
```

### 注意事项

板卡上本地编译需要的时间较长，rk3576、rk3588系列可能需要半个小时，rk3528、rk356x系列可能需要1个小时。

如果是从sdk获取的内核，编译时报syntax error的错误，可以将内核目录单独复制到家目录或者其他目录再编译。

内核编译成功后方可以继续学习后续内容。
