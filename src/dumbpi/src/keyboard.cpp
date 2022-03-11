#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

bool continue_loop = true;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "dumbpi_keyboard");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("dumbpi_cmd", 1000);
    ros::Rate loop_rate(10); // 100 ms loop

    while (continue_loop && ros::ok())
    {
        char c = getchar();
        if (c == 'b' or c == 'q'){
            continue_loop = false;
        } else if (c == '\n' or c == '\r'){
            continue;
        }   
        std_msgs::String msg;
        msg.data = c;
        ROS_INFO("Keyboard sent: [%s]", msg.data.c_str());

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
