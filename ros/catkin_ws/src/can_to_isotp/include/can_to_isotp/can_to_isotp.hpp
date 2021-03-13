#pragma once

#include <ros/ros.h>

namespace can_to_isotp
{
    class CanToIsotp
    {
        public:
            CanToIsotp(ros::NodeHandle &nodeHandle);

        virtual ~CanToIsotp();

        private:
            ros::NodeHandle* m_nodeHandle;

            void isotpCallBack(/* a definir */);
    };
}