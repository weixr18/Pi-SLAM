# Pi-aSLAM 研究记录

## 1 主动探索

### 1.1 手控+SLAM 测试

#### 1.1.1 测试流程

将小车切换到车载电源后连接电脑。

为了方便，可以写一个usb_cam.launch，内容为usb_cam-test.launch去掉image_view的部分。

首先打开第一个终端，运行usb_cam模块

    roslaunch usb_cam usb_cam.launch

第二个终端，运行ORB-SLAM2

    cd ~/Pi-aSLAM/ORB_SLAM2-master/
    rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Asus.yaml

第三个窗口，运行手控程序

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

测试完成后，需要依次退出node，最后退出roscore，然后关机，然后关闭小车电源。

如果退出正常，应该能在ORB-SLAM2-master/文件夹下看到KeyFrameTrajectory.txt文件。该文件保存了最终计算的相机轨迹，每行分别是：Unix时间戳、3D位置坐标、四元数位姿坐标。

#### 1.1.2 测试获得的信息

1. 初始化需要找一个纹理丰富的区域，小车距离目标约1m，手控小车进行平移，使小车位移1-2个车身长度，然后将摄像头微调至初始目标方向，即可成功初始化。（即：单目初始化必须有平移，不能只有纯旋转。）
2. 单目SLAM的跟踪十分依赖于视野的连续性。以目前的处理速度（1-2FPS）和小车运动速度（500-1000cm/s），小车运动过程中几乎不可能跟踪的上，因此必须使用走走停停策略。
3. 距离障碍物距离小于70cm左右时，转视角会导致视野大幅变化，导致丢失。丢失后视角转回去一般就可以重定位到。
4. 单目SLAM非常依赖纹理和光照。暗处、纹理单一的墙面或家具几乎无法识别特征点。建议使用纹理丰富、颜色鲜艳的各类障碍物。
5. 回环检测没有明确看到，但小车转一圈以后可以识别出一开始的特征点。
6. 比较远的特征点由于没有视差，一般距离估计都很离谱。

## 2 主动闭环

## 3 主动重访
