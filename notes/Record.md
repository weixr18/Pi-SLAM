# Pi-aSLAM 研究记录

## 1 主动探索

### 1.1 手控+SLAM 测试

#### 1.1.1 测试流程

将小车切换到车载电源后连接电脑。

为了方便，可以写一个usb_cam.launch，内容为usb_cam-test.launch去掉image_view的部分。

为了方便起见，可以写一个launch文件来启动所有的节点

    <launch>
        <arg name="path_to_vacabulary" default="/home/pi/Pi-aSLAM/ORB_SLAM2-master/Vocabulary/ORBvoc.txt"/>
        <arg name="path_to_settings" default="/home/pi/Pi-aSLAM/ORB_SLAM2-master/Examples/ROS/ORB_SLAM2/Asus.yaml"/>
        <!-- launch usb_cam -->
        <include file="$(find usb_cam)/launch/usb_cam.launch"/>

        <!--launch ORB-SLAM2 mono -->
        <node name ="Mono" pkg="ORB_SLAM2" type="Mono" 
            args="$(arg path_to_vacabulary) $(arg path_to_settings)" respawn="true" output="screen">
        </node>
    </launch>

打开一个终端窗口，运行launch文件

    roslaunch ORB_SLAM2 ros_mono.launch.launch

打开另一个窗口，运行手控程序

    cd ~/ros_catkin_ws
    bash src/dumbpi/test_move.sh

此时rosnode list应该能看到5个node

    /Mono
    /dumbpi_controller
    /dumbpi_keyboard
    /rosout
    /usb_cam

分别是：单目SLAM，小车驱动，键盘输入，ROS系统输出，相机

然后就可以手控小车SLAM了。

测试完成后，退出roscore，然后关机，然后关闭小车电源。

如果退出正常，应该能在ORB-SLAM2-master/文件夹下看到KeyFrameTrajectory.txt文件。该文件保存了最终计算的相机轨迹，每行分别是：Unix时间戳、3D位置坐标、四元数位姿坐标。

#### 1.1.2 测试获得的信息

1. 初始化需要找一个纹理丰富的区域，小车距离目标约1m，手控小车进行平移，使小车位移1-2个车身长度，然后将摄像头微调至初始目标方向，即可成功初始化。（即：单目初始化必须有平移，不能只有纯旋转。）
2. 单目SLAM的跟踪十分依赖于视野的连续性。以目前的处理速度（1-2FPS）和小车运动速度（500-1000cm/s），小车运动过程中几乎不可能跟踪的上，因此必须使用走走停停策略。
3. 距离障碍物距离小于70cm左右时，转视角会导致视野大幅变化，导致丢失。丢失后视角转回去一般就可以重定位到。
4. 单目SLAM非常依赖纹理和光照。暗处、纹理单一的墙面或家具几乎无法识别特征点。建议使用纹理丰富、颜色鲜艳的各类障碍物。
5. 回环检测没有明确看到，但小车转一圈以后可以识别出一开始的特征点。
6. 比较远的特征点由于没有视差，一般距离估计都很离谱。

### 1.2 主动探索方案

#### 1.2.1 基础方案

近距观察+拉远视野

特征地图转换为2D格点地图

从格点地图估计前方障碍物远近

获取当前帧可见特征点个数

#### 1.2.2 编译流程

首先要把新编写的 Map2d.cc 和 Controller.cc 加入 CMakeLists。把WiringPi加入link library列表。注意主目录和Example/ROS里的都要加。

为了方便，可以写一个buld_light.sh，把编译第三方库和解压dictionary部分去掉，节省时间。此外，可以去掉makefile里非单目的部分。

然后，扩大内存交换区，防止内存不足

    sudo nano /etc/dphys-swapfile

编辑 CONF_SWAPSIZE  变量：从100增加到2048

重新启动交换服务:

    sudo /etc/init.d/dphys-swapfile stop
    sudo /etc/init.d/dphys-swapfile start

然后，进入ORB-SLAM2主目录，开始编译. 注意ROS部分的可执行文件是分开的。

    ./build_light.sh 2>&1
    ./build_ros.sh 2>&1

编译成功后，及时调整交换区大小，避免闪存卡老化。

    sudo nano /etc/dphys-swapfile

编辑 CONF_SWAPSIZE  变量：从2048恢复到100

重新启动交换服务:

    sudo /etc/init.d/dphys-swapfile stop
    sudo /etc/init.d/dphys-swapfile start

#### 1.2.3 测试流程

为了方便使用root权限，可以写一个启动脚本Pi-aSLAM/start_slam.sh

    source /home/pi/ros_catkin_ws/devel/setup.bash
    cd /home/pi/Pi-aSLAM/ORB_SLAM2-master/
    roslaunch ORB_SLAM2 ros_mono.launch

首先打开终端，运行脚本。注意此处**必须使用root权限**，否则驱动无法工作。

    sudo su -c ../start_slam.sh

此时rosnode list应该能看到3个node

    /Mono
    /rosout
    /usb_cam

[BUG.1] 新写的Controller线程不输出

查看日志发现系统其他线程均输出正常。
实际原因：sleep函数在linux上以秒而不是毫秒为单位。

[BUG.2] Controller控制车轮不转
实际原因：sleep只接受整数输入，毫秒级别延迟需使用msleep。该函数需要添加unistd.h头文件。

[Bug.3] 重建2D地图时闪退bug
问题原因：非重建置零时使用错误的矩阵清零方式，导致长周期指针指向短周期对象，销毁后内存泄漏。修改后恢复正常。

#### 1.2.4 录制视频流程

1. 启动程序，等待加载ORB词典
2. 开始SLAM后立刻调整图像框位置
3. 电脑先切vscode再切VNC，win+G开始录屏
4. 开始手持手机录像

### 1.3 2D地图调试

[BUG.4] 障碍判定不鲁棒，容易被错误特征点迷惑导致障碍过近
问题原因：经初步分析，至少有如下几个问题

1. 目前的障碍仅仅基于特征点的投影，并没有进一步进行滤波，无法排除干扰
2. 目前的投影并不是按照严格水平面投影，而是和真实平面有一个夹角，导致地图不准
3. 目前障碍仅考虑相机镜头左右±15°的障碍点，范围不够大

解决方法：首先增大搜索范围。其次，考虑使用二值运算进行滤波。

## 2 主动重访

### 3.1 获取地图点的BA误差

在LocalMapping线程中，修改了调用局部BA的逻辑，使BA后保留optimizer对象。从该对象中优化图的边中即可获得每个被优化地图点的BA误差。

#### 3.1.1 重编译g2o库

为了获取图优化后每条边的误差，修改了g2o库的头文件。修改完需要重新编译g2o库

在build_light.sh的开头加入：

    echo "Configuring and building Thirdparty/g2o ..."
    cd Thirdparty/g2o
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j4
    cd ../../..

然后运行build_light.sh 即可重新编译g2o

#### 3.1.2 其他错误

[BUG.5] Map2d.h包含eigen头文件出错。

    fatal error: Eigen/core: No such file or directory
    #include "Eigen/core"

解决方法：大小写出错，应为 #include "Eigen/Core"

[BUG.6] 未知原因闪退

问题定位：获取误差平方后尝试调用鲁棒核函数，但边可能未设置核函数
解决方法：判断核函数是否设置，若设置则调用，否则直接使用平方值
是否解决：是

### 3.2 建立error map

#### 3.2.1 重定义Map2d数据结构

将原有基于opencv的Mat的数据结构修改为基于结构体二维数组的结构，使用更加简单。

[BUG.7] 未知原因崩溃

报错如下，或无报错闪退。

    OpenCV(3.4.3) /home/pi/Downloads/opencv-3.4.3/modules/core/src/matrix.cpp:405: error: (-215:Assertion failed) m.dims >= 2 in function 'Mat'

问题原因：新建map时没有给成员变量赋值而是给一个局部变量赋了值，导致成员变量未初始化。
解决方法：改成给成员变量赋值
是否解决：部分

#### 3.2.2 添加障碍点滤波函数

[BUG.8] 障碍点滤波过程中未知原因崩溃
问题定位：生成二值图像函数矩阵维度问题。
问题原因：cv::Mat::zeros参数顺序是行数、列数、数据类型。
要使(0,0)在图像左下角，行数应为z方向范围，列数应为x方向范围。
同时使用at<>访问元素，形参顺序是行号、列号。行号应为z方向范围-z方向索引-1，列号应为x方向索引。
是否解决：部分

[BUG.9] 障碍点滤波过程中报错崩溃

    OpenCV(3.4.3) /home/pi/Downloads/opencv-3.4.3/modules/imgproc/src/connectedcomponents.cpp:3928: error: (-215:Assertion failed) connectivity == 8 || connectivity == 4 in function 'connectedComponents_sub1'

问题原因：调用connectedComponentsWithStats函数时少传了connectivity参数
是否解决：部分

[BUG.10] 障碍点滤波过程中未知原因崩溃
问题原因：局部cv::Mat类型变量访问元素时使用的at类型参数不对，且行号未-1
是否解决: 是

[BUG.11] 障碍点滤波过程中背景被全部选中
问题原因：选取连通域时未排除最大连通域
解决方法：判断连通域外接矩形和全图大小关系，若大小接近则不选中
是否解决：是

#### 3.2.3 error map算法

### 3.3 根据error map路径规划

### 3.4 路径规划的执行

## 3 其他主动SLAM

### 3.1 主动闭环

### 3.2 主动重定位
