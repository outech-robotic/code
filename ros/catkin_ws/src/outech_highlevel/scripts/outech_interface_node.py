#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg as std_msgs
from lib.proto.gen.python.outech_pb2 import BusMessage, ServoMsg

def outech_interface_node():
    servo_msg = BusMessage(servo=ServoMsg())
    servo_msg.servo.id = 0
    servo_msg.servo.angle = 45
    direction = 4
    multiarray = std_msgs.UInt8MultiArray(layout=std_msgs.MultiArrayLayout(dim=[std_msgs.MultiArrayDimension(label="serialized", size=32, stride=32)], data_offset=0), data=[])
    pub = rospy.Publisher('pb_test', std_msgs.UInt8MultiArray, queue_size=10)
    rospy.init_node('outech_interface', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        servo_msg.servo.angle += direction
        if servo_msg.servo.angle < 30 or servo_msg.servo.angle > 150:
            direction = -direction
        buffer = servo_msg.SerializeToString()
        print(buffer, len(buffer))
        multiarray.data = buffer
        multiarray.layout.dim[0].stride = multiarray.layout.dim[0].size = len(buffer)
        pub.publish(multiarray)
        rate.sleep()

if __name__ == '__main__':
    try:
        outech_interface_node()
    except rospy.ROSInterruptException:
        pass