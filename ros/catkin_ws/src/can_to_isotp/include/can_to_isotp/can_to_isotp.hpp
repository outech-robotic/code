#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "can_msgs/Frame.h"

namespace can_to_isotp
{
    class CanToIsotp
    {
        public:
            CanToIsotp(ros::NodeHandle &node_handle);

        virtual ~CanToIsotp();

        private:
            ros::NodeHandle& m_node_handle;
            ros::Subscriber m_can_subscriber;
            ros::Publisher m_can_publisher;

        void isotpCallBack(const can_msgs::Frame &msg);
    };
}