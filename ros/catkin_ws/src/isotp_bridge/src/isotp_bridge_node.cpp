#include <ros/ros.h>
#include "isotp_bridge/isotp_bridge.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "isotp_bridge");
    ros::NodeHandle nodeHandle("~");

    isotp_bridge::ISOTPBridge isotp_bridge_node(nodeHandle);

    ros::spin();
    return 0;
}