#include "can_to_isotp/can_to_isotp.hpp"
#include <cmath>

namespace can_to_isotp
{
    CanToIsotp::CanToIsotp(ros::NodeHandle &nodeHandle) :
        m_nodeHandle(&nodeHandle)
    {
        std::string canTopicName = "received_messages";
        if (!this->m_nodeHandle->getParam("can_topic_name", canTopicName));
        {
            ROS_ERROR("Load can_topic_name param fail!");
            //return;
        }

        this->m_canSubscriber = this->m_nodeHandle->subscribe(canTopicName, 10, &CanToIsotp::isotpCallBack, this);
    }

    CanToIsotp::~CanToIsotp()
    {
    }

    void CanToIsotp::isotpCallBack(const std_msgs::String::ConstPtr& msg)
    {
        /* CODE DE LA LIB ISO TP */

        ROS_INFO_STREAM("Test!"/*Mettre la frame ISO TP reconstruite ici pour verif */);
    }
}