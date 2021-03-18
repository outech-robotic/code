#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <std_msgs/UInt8MultiArray.h>
#include "isotpc/isotp.h"

namespace isotp_bridge
{
    class ISOTPBridge
    {
        public:
            ISOTPBridge(ros::NodeHandle &node_handle);
            ~ISOTPBridge();

        private:
            ros::NodeHandle &m_node_handle;
            ros::Subscriber m_can_subscriber;
            ros::Publisher m_can_publisher;
            ros::Subscriber m_user_subscriber;
            ros::Publisher m_user_publisher;

            /*
             * ISO-TP library related elements
             */ 
            IsoTpLink m_isotp_link;
            const uint16_t m_rx_addr = 0;
            const uint16_t m_tx_addr = 0;
            uint8_t m_isotp_rx_buffer[4096];
            uint8_t m_isotp_tx_buffer[4096];

            /*
             * Callbacks on subscribed topics
             */
            // Connected to ISO-TP enabled CAN bus topic (socketcan_bridge for example)
            void can_interface_callback(const can_msgs::Frame &msg);
            // Connected to anything that needs to send buffers through CAN
            void user_interface_callback(const std_msgs::UInt8MultiArray &msg);
    };
}