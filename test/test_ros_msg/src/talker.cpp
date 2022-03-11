
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    
    //必须在最开始调用ros::init()。ros::init() 函数需要接受argc和argv。
    ros::init(argc, argv, "talker");

    // NodeHandle是node和ros通信的入口，第一个NodeHandle的构造会完整初始化node，
    // 最后一个NodeHandle的析构会关闭node。
    ros::NodeHandle n;

    /**
     * 用advertise() 建立一个topic并返回一个publisher。可以调用
     * 其publish()方法进行具体的通信。如果所有publisher和其copy都被析构，
     * 那么该topic会结束。 参数1是topic的名字，参数2是缓冲区的大小。
     */
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // ros::Rate ros自带计时器，10代表10Hz
    ros::Rate loop_rate(10);
    int count = 0;

    // ros::ok ???
    while (ros::ok())
    {
        //msg object
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        // ROS 控制台回显
        ROS_INFO("%s", msg.data.c_str());
        // 调用 publish()
        chatter_pub.publish(msg);
        // 调用topic的回调函数
        ros::spinOnce();
        // 计时器沉默一阵子
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
