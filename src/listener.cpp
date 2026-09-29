#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Listener] Received: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

    ROS_INFO("[Listener] Waiting for incoming messages on /chatter...");
    ros::spin();

    return 0;
}
