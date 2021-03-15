#include "can_to_isotp/can_to_isotp.hpp"
#include "isotpc/isotp.h"

namespace can_to_isotp
{
    CanToIsotp::CanToIsotp(ros::NodeHandle &node_handle) :
        m_node_handle(node_handle)
    {
        std::string topic_subscribed = "/received_messages";
        std::string topic_published = "/sent_messages";

        ROS_INFO("Subscribing to topic:%s", topic_subscribed.c_str());
        ROS_INFO("Publishing to topic:%s", topic_published.c_str());

        this->m_can_subscriber = this->m_node_handle.subscribe(topic_subscribed, 10, &CanToIsotp::isotpCallBack, this);
        this->m_can_publisher = this->m_node_handle.advertise<can_msgs::Frame>(topic_published, 10);
    }

    CanToIsotp::~CanToIsotp()
    {
    }

    void CanToIsotp::isotpCallBack(const can_msgs::Frame &msg)
    {
        /*print la frame ISO TP reconstruite ici pour verif */
        ROS_INFO("Recu:\nid:0x%X\nbytes:%d ", msg.id, msg.dlc);
    }
}