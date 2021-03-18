#include <stdint.h>
#include <ros/ros.h>
#include "can_msgs/Frame.h"

extern ros::Publisher *g_can_publisher;

#ifdef __cplusplus
extern "C" {
#endif

/* user implemented, print debug message */
void isotp_user_debug(const char* message, ...)
{
    ROS_DEBUG("ISOTP DEBUG:%s", message);
}

/* user implemented, send can message */
int  isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint8_t size)
{
    
    return 0;
}

/* user implemented, get millisecond */
uint32_t isotp_user_get_ms(void)
{
    return 0;
}

#ifdef __cplusplus
}
#endif
