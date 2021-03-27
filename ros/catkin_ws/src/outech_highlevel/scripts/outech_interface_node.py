#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg as std_msgs
from lib.proto.gen.python.outech_pb2 import BusMessage

def outech_interface_node():
    pub = rospy.Publisher('pb_test', std_msgs.String, queue_size=10)
    rospy.init_node('outech_interface', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        outech_interface_node()
    except rospy.ROSInterruptException:
        pass