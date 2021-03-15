#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>

#include "isotpc/isotp.h"

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

            IsoTpLink m_isotp_link;
            const uint16_t m_isotp_rx_addr = 0;
            const uint16_t m_isotp_tx_addr = 0;
            uint8_t m_isotp_rx_buffer[4096];
            uint8_t m_isotp_tx_buffer[4096];

            void isotpCallBack(const can_msgs::Frame &msg);
    };
}