#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "dumbpi_keyboard");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("dumbpi_cmd", 1000);

    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "hello, world" + std::to_string(count);
        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
