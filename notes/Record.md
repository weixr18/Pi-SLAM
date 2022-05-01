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

#### 1.2.4 录制视频流程

1. 启动程序，等待加载ORB词典
2. 开始SLAM后立刻调整图像框位置
3. 电脑先切vscode再切VNC，win+G开始录屏
4. 开始手持手机录像

### 1.3 带反馈的调速轮

为了解决电机扭矩不够导致小车只能断续前进的问题，升级了小车电机，并重写了控制方案。

## 2 主动闭环

## 3 主动重访
