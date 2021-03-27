#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg as std_msgs
from lib.proto.gen.python.outech_pb2 import BusMessage, ServoMsg
from outech_highlevel.msg import Servo


motor_publisher = None
servo_publishers = []

def callback_servo(servomsg):
    print(servomsg)
    servo_msg = BusMessage(servo=ServoMsg())
    servo_msg.servo.id = servomsg.servo_id
    servo_msg.servo.angle = servomsg.angle

    buffer = servo_msg.SerializeToString()
    servo_msg = BusMessage(servo=ServoMsg())
    multiarray = std_msgs.UInt8MultiArray(
    layout=std_msgs.MultiArrayLayout(dim=[std_msgs.MultiArrayDimension(label="serialized", size=0, stride=0)], data_offset=0),
    data=[])

    multiarray.data = buffer
    multiarray.layout.dim[0].stride = multiarray.layout.dim[0].size = len(buffer)

    servo_publishers[servomsg.board_id].publish(multiarray)

def outech_interface_node():
    rospy.init_node('outech_interface', anonymous=True)
    
    motor_publisher = rospy.Publisher('bridge_motor/sent_messages', std_msgs.UInt8MultiArray, queue_size=10)
    servo_publishers.append(rospy.Publisher('bridge_servo_0/sent_messages', std_msgs.UInt8MultiArray, queue_size=10))
    servo_publishers.append(rospy.Publisher('bridge_servo_1/sent_messages', std_msgs.UInt8MultiArray, queue_size=10))
    servo_publishers.append(rospy.Publisher('bridge_servo_2/sent_messages', std_msgs.UInt8MultiArray, queue_size=10))

    rospy.Subscriber("servo_msg", Servo, callback_servo)

    rospy.spin()


if __name__ == '__main__':
    try:
        outech_interface_node()
    except rospy.ROSInterruptException:
        pass
