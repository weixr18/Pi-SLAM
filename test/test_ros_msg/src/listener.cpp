#include "ros/ros.h"
#include "std_msgs/String.h"

//回调函数
void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    /**
     * 用subscribe() 订阅一个topic并返回一个subscriber，以名字作为标识符。
     * 参数1是topic的名字，参数2是缓冲区的大小，参数3是接收到消息后的回调函数。
     */
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    
    // 停止主程序运行，并开启回调函数，此时回调函数可以正常工作。
    ros::spin();
    
    return 0;
}
