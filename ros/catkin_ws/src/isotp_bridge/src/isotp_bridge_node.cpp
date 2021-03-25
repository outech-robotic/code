#include <ros/ros.h>
#include "isotp_bridge/isotp_bridge.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "isotp_bridge");
    ros::NodeHandle nodeHandle("~");

    isotp_bridge::ISOTPBridge isotp_bridge_node(nodeHandle);

    ros::Rate loop_rate(2000);
    while(ros::ok())
   {
        isotp_bridge_node.update_state();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}