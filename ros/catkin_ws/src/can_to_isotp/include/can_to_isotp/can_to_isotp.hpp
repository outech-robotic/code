#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"
namespace can_to_isotp
{
    class CanToIsotp
    {
        public:
            CanToIsotp(ros::NodeHandle &nodeHandle);

        virtual ~CanToIsotp();

        private:
            ros::NodeHandle* m_nodeHandle;
            ros::Subscriber m_canSubscriber;

            void isotpCallBack(const std_msgs::String::ConstPtr& msg);
    };
}