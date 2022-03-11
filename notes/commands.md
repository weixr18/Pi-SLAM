# commands

## 快捷键

Ctrl+Shift+T 终端

## 命令

### 系统相关

#### 系统设置

    sudo raspi-config

#### 重新启动

    sudo shutdown -r now

#### 查看系统信息

    uname -a
    cat /proc/vesion
    cat /etc/issue
    cat /etc/os-release
    lsb_release -a
    screenfetch

#### 查看磁盘占用情况

    df -h

#### 包管理器和系统更新

    sudo apt-get update
    sudo apt-get upgrade

#### 查看CPU温度

    cat /sys/class/thermal/thermal_zone0/temp

返回值除以1000是摄氏度温度

## 文件操作

### tar

压缩

    tar -czvf myetc.tar.gz etc

解压

    tar -xzvf myetc.tar.gz etc

### zip unzip

    zip -r name.zip dir/name

## 进程操作

### 查找进程

    ps aux | grep "name"
