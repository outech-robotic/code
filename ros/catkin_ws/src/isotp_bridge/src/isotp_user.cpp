#include <stdint.h>
#include <ros/ros.h>
#include <time.h>
#include <can_msgs/Frame.h>

extern ros::Publisher *g_can_publisher;

#ifdef __cplusplus
extern "C"{
#endif

/* user implemented, print debug message */
void isotp_user_debug(const char* message, ...)
{
    ROS_INFO("ISOTP DEBUG:%s", message);
}

/* user implemented, send can message */
int  isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint8_t size)
{
    static uint32_t can_seq_number = 0;

    can_msgs::Frame frame;

    frame.header.seq = can_seq_number++;
    frame.header.stamp = ros::Time::now();
    frame.header.frame_id = "/base_link";

    frame.id = arbitration_id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    frame.dlc = size;
    for(uint8_t i = 0; i<size; i++){
        frame.data[i]=data[i];
    }
    g_can_publisher->publish(frame);
    return 0;
}

/* user implemented, get millisecond */
uint32_t isotp_user_get_ms(void)
{
    return ros::Time::now().toNSec()/1000000;
}

#ifdef __cplusplus
}
#endif
