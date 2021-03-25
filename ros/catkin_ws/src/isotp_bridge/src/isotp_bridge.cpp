#include "isotp_bridge/isotp_bridge.hpp"
#include "isotpc/isotp_defines.h"
#include <iostream>

ros::Publisher *g_can_publisher;

namespace isotp_bridge
{
    ISOTPBridge::ISOTPBridge(ros::NodeHandle &node_handle) :
        m_node_handle(node_handle)
   {
        const std::string &topic_subscribed = "received_messages";
        const std::string &topic_published = "sent_messages";

        /*
         * Initializes publishers and receivers
         */
        // CAN interface
        this->m_can_subscriber = this->m_node_handle.subscribe("/received_messages", 10, &ISOTPBridge::can_interface_callback, this);
        this->m_can_publisher = this->m_node_handle.advertise<can_msgs::Frame>("/sent_messages", 10);
        this->m_user_subscriber = this->m_node_handle.subscribe("sent_messages", 10, &ISOTPBridge::user_interface_callback, this);
        this->m_user_publisher = this->m_node_handle.advertise<std_msgs::UInt8MultiArray>("received_messages", 10);
        g_can_publisher = &this->m_can_publisher; // for ISOTP-C library use


        /*
         * Get CAN IDs for SocketCAN from ROS Parameter Server
         */
        int board_id;
        if(m_node_handle.getParam("board_id", board_id)){
            ROS_INFO("Initializing isotp_bridge for board ID: %u", board_id);
        }
        else{
            ROS_WARN("Initializing isotp_bridge for DEFAULT board ID: 0");
            board_id = 0;
        }

        if(2*board_id+1 > 0x7FF){
            ROS_ERROR("CAN Address needs to fit in 11bits");
            ros::shutdown();
        }
        m_tx_addr = 2*board_id;
        m_rx_addr = 2*board_id + 1;

        /*
         * Initializes the ISOTP-C library and gives it its buffers
         */
        isotp_init_link(&m_isotp_link, m_tx_addr, m_isotp_tx_buffer, sizeof(m_isotp_tx_buffer), m_isotp_rx_buffer, sizeof(m_isotp_rx_buffer));
    }

    ISOTPBridge::~ISOTPBridge()
   {
        g_can_publisher = nullptr;
    }

    void ISOTPBridge::can_interface_callback(const can_msgs::Frame &msg_received)
   {   
        if(msg_received.id == m_rx_addr){
            std_msgs::UInt8MultiArray msg_to_user;
            uint8_t rx_data[8];
            uint8_t received_frame[4096];
            uint16_t received_size;

            // Copy the data to reception buffer
            for(uint8_t i = 0; i<8; i++){
                rx_data[i]=msg_received.data[i];
            }

            // Update ISOTP reception state
            isotp_on_can_message(&m_isotp_link, rx_data, msg_received.dlc);

            // Update ISOTP state machine
            isotp_poll(&m_isotp_link);

            // Read received buffer if complete
            if(isotp_receive(&m_isotp_link, received_frame, sizeof(received_frame), &received_size) == ISOTP_RET_OK){
                // Setup message to publish to user interface
                msg_to_user.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg_to_user.layout.dim[0].label = "buffer";
                msg_to_user.layout.dim[0].size = received_size;
                msg_to_user.layout.dim[0].stride = received_size;
                msg_to_user.layout.data_offset = 0;
                msg_to_user.data.resize(received_size);
                for(uint16_t i = 0; i<received_size; i++){
                    msg_to_user.data[i] = received_frame[i];
                }

                m_user_publisher.publish(msg_to_user);
            }
        }
    }

    void ISOTPBridge::user_interface_callback(const std_msgs::UInt8MultiArray &msg_to_send)
   {
        can_msgs::Frame msg_to_can;
        std::cout << "Received user buffer:";
        for(auto &data : msg_to_send.data){
                std::cout << std::hex << std::setfill('0') << std::setw(2) << data;
        }
        std::cout<<std::endl;

        ROS_INFO("Size:%d", msg_to_send.layout.dim[0].stride);
        if(isotp_send(&m_isotp_link, msg_to_send.data.data(), msg_to_send.layout.dim[0].stride) != ISOTP_RET_OK){
            ROS_WARN("Error: Failed to send ISOTP message");
        }

    }

    void ISOTPBridge::update_state()
   {   
        static bool was_sending = false;
        isotp_poll(&m_isotp_link);
        if(m_isotp_link.send_status == ISOTP_SEND_STATUS_INPROGRESS)
       {   
            was_sending = true;
            ROS_INFO("ISOTP SEND IN PROGRESS");
        }
        else{   
            if(was_sending){
                ROS_INFO("ISOTP SEND NOT IN PROGRESS");
                if(m_isotp_link.send_status == ISOTP_SEND_STATUS_ERROR){
                    ROS_WARN("ISOTP SEND ERROR");
                }
            }
            was_sending = false;
        }
    }
}