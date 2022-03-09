# Pi-aSLAM 全记录

## 1 起步工作

### 1.1 SD卡烧录

如果树莓派出现红灯常亮，绿灯闪烁几下后不亮，拔下SD卡后绿灯有规律闪烁，则说明SD卡出现问题，需要重新烧录。因此建议对SD卡进行全盘备份。

烧写参考 [https://blog.csdn.net/ourkix/article/details/113412367](https://blog.csdn.net/ourkix/article/details/113412367)

进行烧写需要SD卡格式化工具（系统自带工具即可）和映像写入工具（可用win32 image writer）

### 1.2 系统初始化

该过程需要从树莓派外接键鼠、显示屏。

#### 1.2.1 显示屏需要设置驱动

参考链接：[https://blog.csdn.net/weixin_30045751/article/details/114020270](https://blog.csdn.net/weixin_30045751/article/details/114020270)

重要：**显示屏需要从mini-HDMI口连接**

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

电脑 VNC Viewer，注意IP地址

## 2 软硬件配置调试

### 2.1 换源

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

### 2.5 麦克纳姆轮驱动

2. wiringPi目前不支持树莓派4B，需要手动更新
3. 需要运动时注意树莓派和小车其他部分共地
