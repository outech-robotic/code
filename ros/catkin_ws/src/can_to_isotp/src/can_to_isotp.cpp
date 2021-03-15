#include "can_to_isotp/can_to_isotp.hpp"
#include <iostream>

ros::Publisher *g_can_publisher;

namespace can_to_isotp
{
    CanToIsotp::CanToIsotp(ros::NodeHandle &node_handle) :
        m_node_handle(node_handle)
    {
        std::string topic_subscribed_can = "/received_messages";
        std::string topic_published_can = "/sent_messages";

        ROS_INFO("Subscribing to topic:%s", topic_subscribed_can.c_str());
        ROS_INFO("Publishing to topic:%s", topic_published_can.c_str());

        this->m_can_subscriber = this->m_node_handle.subscribe(topic_subscribed_can, 10, &CanToIsotp::isotpCallBack, this);
        this->m_can_publisher = this->m_node_handle.advertise<can_msgs::Frame>(topic_published_can, 10);
        g_can_publisher = &this->m_can_publisher;

        isotp_init_link(&m_isotp_link, m_isotp_tx_addr, m_isotp_tx_buffer, sizeof(m_isotp_tx_buffer), m_isotp_rx_buffer, sizeof(m_isotp_rx_buffer));
    }

    CanToIsotp::~CanToIsotp()
    {

    }

    void CanToIsotp::isotpCallBack(const can_msgs::Frame &msg_received)
    {   
        can_msgs::Frame msg_to_send;
        
        uint8_t received_frame[4096] = {0};
        uint16_t received_size;
        
        uint8_t rx_data[8];
        for(int i = 0; i<msg_received.dlc; i++){
            rx_data[i]=msg_received.data[i];
        }

        ROS_INFO("Recu:\nid:0x%X\nbytes:%d ", msg_received.id, msg_received.dlc);       
        
        isotp_on_can_message(&m_isotp_link, rx_data, msg_received.dlc);

        isotp_poll(&m_isotp_link);

        if (isotp_receive(&m_isotp_link, received_frame, sizeof(received_frame), &received_size) == ISOTP_RET_OK) {
            ROS_INFO("Recu trame:%d", received_size);
            for(int i=0; i<received_size; i++){
                std::cout << std::hex << std::setfill('0') << std::setw(2) << received_frame[i];
            }            
            std::cout<<std::endl;
        }

    }
}