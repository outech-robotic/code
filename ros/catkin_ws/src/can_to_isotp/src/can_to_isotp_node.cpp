#include <ros/ros.h>
#include "can_to_isotp/can_to_isotp.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_to_isotp");
    ros::NodeHandle nodeHandle("~");

    can_to_isotp::CanToIsotp canToIsotp(nodeHandle);

    ros::spin();
    return 0;
}