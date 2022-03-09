# Pi-aSLAM 全记录

## 1 起步工作

### 1.1 SD卡烧录

如果树莓派出现红灯常亮，绿灯闪烁几下后不亮，拔下SD卡后绿灯有规律闪烁，则说明SD卡出现问题，需要重新烧录。因此建议对SD卡进行全盘备份。

烧写参考 [https://blog.csdn.net/ourkix/article/details/113412367](https://blog.csdn.net/ourkix/article/details/113412367)

进行烧写需要SD卡格式化工具（系统自带工具即可）和映像写入工具（可用win32 image writer）

### 1.2 系统初始化

该过程需要从树莓派外接键鼠、显示屏。

#### 1.2.1 显示屏设置

在VNC连接前，**显示屏需要从mini-HDMI口连接**

为了正常显示，需要修改SD卡固件部分的config.txt，确保以下几项没有被注释掉。

    hdmi_force_hotplug=1
    config_hdmi_boost=4
    hdmi_group=2
    hdmi_mode=16
    hdmi_drive=2

参考链接：[https://blog.csdn.net/weixin_30045751/article/details/114020270](https://blog.csdn.net/weixin_30045751/article/details/114020270)

注意：固件中设置的hdmi_mode会覆盖系统里的设置。如果想在系统中改变分辨率，需要把固件的这行注释掉。

#### 1.2.2 系统初始化

用户名是pi（密码自己定，我的123456）

Ctrl+Shift+T 可以打开terminal

#### 1.2.3 打开SSH

    sudo raspi-config

3 Interface config 找到SSH，打开

#### 1.2.4 打开VNC server

同上

    sudo raspi-config

3 Interface config 找到VNC设置，开启

显示分辨率的设置在2 Display Settings

#### 1.2.5 树莓派重启

    sudo shutdown -r now

### 1.3 VNC连接

#### 1.3.1 找到树莓派IP地址

确保以下设置无误：

1. wifi的适配器设置，共享页面，允许其他设备连接和控制
2. 进一步设置里允许1703服务
3. 重新设置后拔掉网线重新插入

#### 1.3.2 VNC连接

电脑需要先安装VNC Viewer。打开，填写用户名密码，注意IP地址。

## 2 软硬件配置调试

### 2.1 系统换源

    sudo nano /etc/apt/sources.list

加这两行

    deb http://mirrors.tuna.tsinghua.edu.cn/raspbian/raspbian/ buster main non-free contrib
    deb-src http://mirrors.tuna.tsinghua.edu.cn/raspbian/raspbian/ buster main non-free contrib

然后

    sudo apt-get uodate

换源完可以安装一个vim试一下

    sudo apt-get install vim

### 2.2 安装opencv v3

#### 2.2.1 准备工作

清理不需要的大型软件

    sudo apt-get purge wolfram-engine
    sudo apt-get purge libreoffice*
    sudo apt-get clean
    sudo apt-get autoremove

系统更新

    sudo apt-get upgrade

#### 2.2.2 安装必要工具

安装build-essential、cmake、git和pkg-config

    sudo apt-get install build-essential cmake git pkg-config

安装图像工具包

    sudo apt-get install libjpeg8-dev
    sudo apt-get install libtiff5-dev
    sudo apt-get install libjasper-dev
    sudo apt-get install libpng12-dev 

安装视频工具包

    sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

安装GTK

    sudo apt-get install libgtk2.0-dev

安装数值优化工具包

    sudo apt-get install libatlas-base-dev gfortran

#### 2.2.3 下载源码和解压

下载Opencv 3.4.3

    wget -O opencv-3.4.3.zip https://github.com/Itseez/opencv/archive/3.4.3.zip

解压OpenCV

    unzip opencv-3.4.3.zip

下载OpenCV_contrib库：

    wget -O opencv_contrib-3.4.3.zip https://github.com/Itseez/opencv_contrib/archive/3.4.3.zip

解压OpenCV_contrib库：

    unzip opencv_contrib-3.4.3.zip

#### 2.2.4 编译前配置

建立目录

    cd ~/opencv-3.4.3
    mkdir build
    cd build

cmake设置

    /** CMAKE_BUILD_TYPE是编译方式
    * CMAKE_INSTALL_PREFIX是安装目录
    * OPENCV_EXTRA_MODULES_PATH是加载额外模块
    * INSTALL_PYTHON_EXAMPLES是安装官方python例程
    * BUILD_EXAMPLES是编译例程（这两个可以不加，不加编译稍微快一点点，想要C语言的例程的话，在最后一行前加参数INSTALL_C_EXAMPLES=ON，要C++例程的话在最后一行前加参数INSTALL_C_EXAMPLES=ONINSTALL_CXX_EXAMPLES=ON）
    **/
    
    sudo cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.4.3/modules \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D INSTALL_CXX_EXAMPLES=ON \
        -D BUILD_EXAMPLES=ON ..

备份build文件

因为下一步的编译会使用build文件中的东西，假如编译失败后还要重新进行cmake，比较耽误时间，这里可以直接备份一下cmake好的build文件夹，命名为build1，重新make的时候可以拿来用。

    cd ..
    cp -r build ./build1

#### 2.2.5 xfeatures2d相关文件

有11个文件需要在上一步下载，但一般会下载失败，需要手动导入opencv_contrib/modules/xfeatures2d/src/这个路径

#### 2.2.6 为树莓派增加SWAP

在开始编译之前，建议你增加交换空间。这将使你使用树莓派的所有四个内核来编译OpenCV，而不会由于内存耗尽导致编译挂起。

    sudo nano /etc/dphys-swapfile

然后编辑 CONF_SWAPSIZE  变量：从100增加到2048

重新启动交换服务:

    sudo /etc/init.d/dphys-swapfile stop
    sudo /etc/init.d/dphys-swapfile start

#### 2.2.7 开始编译

    sudo make -j4 2>&1 | tee make.log

编译需要至少30分钟，可能会遇到各种bug，需要从头再来

[BUG.1] 编译opencv-找不到cuda文件问题

报错

    /home/pi/Downloads/opencv-3.4.3/modules/stitching/include/opencv2/stitching/detail/matchers.hpp:52:12: fatal error: opencv2/xfeatures2d/cuda.hpp: No such file or directory
    #  include "opencv2/xfeatures2d/cuda.hpp"

解决方法：在stitching/CMakeList.txt里加入include path，或将该文件中的这个include改为绝对路径。

注意：如果改成绝对路径，编译完就不要急着按2.2.8删掉中间文件，否则会造成orb-slam2编译失败

编译完成后，不要忘记**install**

    sudo make install

以及更新动态链接库

    sudo ldconfig

#### 2.2.8 收尾工作

首先可以用python测试一下是否安装正确

    pi@raspberrypi:~ $ python3
    Python 3.7.3 (default, Jan 22 2021, 20:04:44) 
    [GCC 8.3.0] on linux
    Type "help", "copyright", "credits" or "license" for more information.
    >>> import cv2
    >>> cv2.__version__
    '3.4.3'

要记着把交换区设置改回来（见2.1.6）

    sudo nano /etc/dphys-swapfile

编译文件如果之后不需要就可以删掉了

    cd ../..
    sudo rm -rf opencv* 

### 2.3 安装orb-slam2

#### 2.3.1 安装依赖库

    sudo apt-get install libboost-all-dev libblas-dev liblapack-dev

#### 2.3.2 安装eigen

首先官网下载eigen 3.2.10

    https://gitlab.com/libeigen/eigen/-/releases/3.2.10

下载解压后，进行编译安装

    mkdir build
    cd build
    cmake ..
    make
    sudo make install

#### 2.3.3 安装Pangolin

Pangolin是一个绘图库，orb-slam2用它来画轨迹图。

安装Pangolin前需要使用如下命令安装libglew-dev，不然编译不过。

    sudo apt-get install libglew-dev

从github上下载Pangolin源码。

    https://github.com/stevenlovegrove/Pangolin/releases/tag/v0.5

注意此处**不要下载master分支**，要使用**v0.5分支**！master是开发分支，不稳定，0.6版本也有bug，必须用0.5版本。

在终端中进入源码主目录，并输入以下命令完成Pangoline的编译，安装。

    mkdir build
    cd build
    cmake ..
    make
    sudo make install

[BUG.2] CODEC_FLAG_GLOBAL_HEADER 未声明问题：

```sh
/home/pi/Downloads/Pangolin-0.5/src/video/drivers/ffmpeg.cpp:501:33: error: ‘CODEC_FLAG_GLOBAL_HEADER’ was not declared in this scope
        stream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;
```

解决方法：注释掉Pagolin/src/CMakeList.txt中FFMPEG相关的一段

    #find_package(FFMPEG QUIET)
    #if(BUILD_PANGOLIN_VIDEO AND FFMPEG_FOUND)
    #  set(HAVE_FFMPEG 1)
    #  list(APPEND INTERNAL_INC  ${FFMPEG_INCLUDE_DIRS} )
    #  list(APPEND LINK_LIBS ${FFMPEG_LIBRARIES} )
    #  list(APPEND HEADERS ${INCDIR}/video/drivers/ffmpeg.h)
    #  list(APPEND SOURCES video/drivers/ffmpeg.cpp)
    #  message(STATUS "ffmpeg Found and Enabled")
    #endif()

#### 2.3.4 为树莓派增加SWAP

ORB-SLAM2的编译也必须像2.2.6一样增加虚拟内存，否则会因为内存耗尽导致卡死。

    sudo nano /etc/dphys-swapfile

编辑 CONF_SWAPSIZE  变量：从100增加到2048

重新启动交换服务:

    sudo /etc/init.d/dphys-swapfile stop
    sudo /etc/init.d/dphys-swapfile start

#### 2.3.5 下载ORB_SALM2源码

可以用git clone

    git clone https://github.com/raulmur/ORB_SLAM2.git

为了防止编译时卡死，可以限制一下编译使用的核数。具体来说，打开build.sh文件夹

    cd ORB_SLAM2
    vim ./build.sh

把所有的 'make -j' 改为 'make -j4'即可。

接下来，进行编译

    chmod +x build.sh
    ./build.sh 2>&1 | tee ../build.log

编译耗时较长，至少30分钟，需要耐心等待。

[BUG.3] 静态断言问题

```sh
/home/pi/Pi-aSLAM/ORB_SLAM2-master/src/LoopClosing.cc:438:21:   required from here
/usr/include/c++/8/bits/stl_map.h:122:21: error: static assertion failed: std::map must have the same value_type as its allocator
    static_assert(is_same<typename _Alloc::value_type, value_type>::value,
```

解决方法：将/home/pi/Pi-aSLAM/ORB_SLAM2-master/include/LoopClosing.h中第49-50行

```cpp
typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```

修改为

```cpp
typedef map<KeyFrame* const,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;
```

#### 2.3.6 收尾工作

要把交换区大小及时改回来，否则flash闪存卡寿命很容易迅速到期。

    sudo nano /etc/dphys-swapfile

编辑 CONF_SWAPSIZE  变量：从2048恢复到100

重新启动交换服务:

    sudo /etc/init.d/dphys-swapfile stop
    sudo /etc/init.d/dphys-swapfile start

### 2.4 摄像头调试

#### 2.4.1 系统配置

    sudo raspi-config

3 Interface config 找到Camera，打开

设置完成需要重启生效。

#### 2.4.2 摄像头测试

    sudo ls /dev/video*

如果出现video0设备就是正常的。

    vcgencmd get_camera

如果support和detected都是1就说明正常。

    raspistill -o test_camera.jpg -t 500

如果照片保存成功就说明摄像头正常。-t参数是拍照延时，单位毫秒，默认值好像是5000.

[BUG.4] 找不到摄像头

    mmal: Cannot read camera info, keeping the defaults for OV5647
    mmal: mmal_vc_component_create: failed to create component 'vc.ril.camera' (1:ENOMEM)
    mmal: mmal_component_create_core: could not create component 'vc.ril.camera' (1)
    mmal: Failed to create camera component
    mmal: main: Failed to create camera component
    mmal: Camera is not detected. Please check carefully the camera module is installed correctly

解决方法：检查一下接线是否松动，如果松动就重新接线，然后重启树莓派，再次尝试就可以了。

### 2.5 麦克纳姆轮测试

#### 2.5.1 更新wiringPi

由于官方系统自带的wiringPi目前不支持树莓派4B，需要手动更新。

先查看树莓派 gpio 版本

    gpio -v

如果是2.50就需要更新

    wget https://project-downloads.drogon.net/wiringpi-latest.deb
    sudo dpkg -i wiringpi-latest.deb

再次检查gpio版本，如果是2.52就可以了。

#### 2.5.2 工作台准备

为准备车轮测试，需要**将小车抬高，使四轮悬空**。否则小车可能在通电一瞬间运动起来，冲出桌面，产生结构损坏。

#### 2.5.3 电源切换

接下来的测试需要将电源从外接电源**切换至车载电源**。这是因为需要运动时，树莓派和小车驱动电源部分需要共地，外接电源无法保证共地（我自己搭的车载电源电路是共地的）。

安装车载电池时，一定要确保**总电源开关处于关闭状态**，否则可能产生电火花或意料之外的情况。

电池安装完毕后，先**用万用表测试电压**是否在标准范围内（10.5-12V）。若电压过高或过低有可能损坏树莓派。如果电压不足，需要充电。

测试无误后，先**不接树莓派电源**，尝试开启总开关，观察各电路板状态以及车轮是否会异常转动。正常情况下，上面板的降压和下面板的两个PWM驱动都应该亮红灯，表示电源正常。此外，如果车轮高速转动或有异响，可能说明电路有问题。

#### 2.5.4 测试

使用./src/test/move中的代码。

    make
    sudo ./test_wheels

此处执行必须要使用superuser身份，因为GPIO驱动需要系统调用，需要较高权限。

正常情况下，每个轮子都应该朝前转动。如果方向有问题，那么有以下几种解决方法：

+ 更改驱动代码
+ 重新连接驱动板的方向控制线
+ 重新连接电机到驱动板的线

### 2.6 麦克纳姆轮驱动

下面简单介绍一下车轮驱动的原理。

#### 2.6.1 L298N芯片

小车的车轮连着电机。我们需要让电机按照我们希望的速度和方向旋转，就需要通过PWM信号控制电机。电机不能直接接收PWM信号，需要一个芯片来进行翻译，这个芯片就是L298N。

L298N芯片自带PCB板，板上接的线分为电源和信号。电源的输入要求5-46V直流电，一般使用12V。输出是两组，可接两个马达。由于是PWM，不分正负，只有正转和反转的区别。

信号部分，两边的两组是使能端。使能端就是PWM信号，通过高电平占比来控制转速。使能端默认有跳帽连5V信号，即占比100%。我们会使用使能端来控制转速。

信号余下的四个脚是逻辑输入，IN1/IN2控制A输出，IN3/IN4控制B输出。控制一组输出的两个脚，1/0和0/1分别是两个方向的转动（需要自己试一下正反）。1/1是刹车制动，0/0是不控制。

因此，每个轮子的控制需要两根方向+一根PWM，总共4*3=12根线。

#### 2.6.2 异或门升压

树莓派GPIO端口输出的电平是3.3V，而驱动L298N需要5V。因此我们需要用5V的逻辑门来进行升压。实际中我使用了SN74HC86N(CMOS异或门)接地实现同门来进行升压。由于有12根输入线，一共需要12个非门，即3个74HC86N芯片。

[BUG.5] 同或门不工作问题：有时候会遇到输入高电平输出为0V，这是因为电源电压太高（5.10V就会出问题），使得3.3V左右的输入高电平被误认为是低电平。此时调整电源电压即可。

#### 2.6.3 麦克纳姆轮

麦克纳姆轮是一种可以让四轮车实现原地转弯和平移的轮子，但使用中需要改变轮子的旋转方向。

具体来说，前进时所有轮子均向前，后退时所有轮子均向后，和普通小车一样。在平移时，平移方向一侧轮子俯视向内对转，另一侧两轮俯视向外对转。原地旋转时 同侧轮方向相同，异侧轮方向相反。

#### 2.6.4 GPIO端口

可以在树莓派中用以下命令查看GPIO信息：

    gpio readall

端口信息：

    +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
    | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
    +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
    |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
    |   2 |   8 |   SDA.1 |   IN | 1 |  3 || 4  |   |      | 5v      |     |     |
    |   3 |   9 |   SCL.1 |   IN | 1 |  5 || 6  |   |      | 0v      |     |     |
    |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | IN   | TxD     | 15  | 14  |
    |     |     |      0v |      |   |  9 || 10 | 1 | IN   | RxD     | 16  | 15  |
    |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
    |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
    |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
    |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
    |  10 |  12 |    MOSI |   IN | 0 | 19 || 20 |   |      | 0v      |     |     |
    |   9 |  13 |    MISO |   IN | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
    |  11 |  14 |    SCLK |   IN | 0 | 23 || 24 | 1 | IN   | CE0     | 10  | 8   |
    |     |     |      0v |      |   | 25 || 26 | 1 | IN   | CE1     | 11  | 7   |
    |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
    |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
    |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
    |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
    |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
    |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
    |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
    +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
    | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
    +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+

实际分配：

| 颜色 | GPIO号 | 物理端口号 | 轮子 | 信号 |
| ------ | ------ | ---------- | ---- | ---- |
| orange | 24     | 35         | FL   | PWM  |
| yellow | 25     | 37         | FR   | PWM  |
| blue   | 27     | 36         | BL   | PWM  |
| green  | 28     | 38         | BR   | PWM  |
| purple | 22     | 31         | FL   | D_f  |
| gray   | 26     | 32         | FL   | D_b  |
| purple | 0      | 11         | FR   | D_f  |
| gray   | 1      | 12         | FR   | D_b  |
| white  | 2      | 13         | BL   | D_f  |
| black  | 3      | 15         | BL   | D_b  |
| brown  | 4      | 16         | BR   | D_f  |
| red    | 5      | 18         | BR   | D_b  |

#### 2.6.5 测试

使用./src/test/move中的代码。

    make
    sudo ./test_move

[BUG.6] 全车接通电源后，进行测试，后左轮和后右轮有时会在停止指令下转动，转动时有时无，后轮的L298N甚至一度发烫，判断为电路短路。经检查，是L298N和不锈钢车板之间绝缘未做好，导致触电短路。充分进行绝缘隔离后，问题解决。
