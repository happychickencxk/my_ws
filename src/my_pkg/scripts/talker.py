#!/usr/bin/env python
from tokenize import String
from sensor_msgs.msg import Image
import rospy
from std_msgs.msg import String

def image_callback():
    rospy.loginfo(rospy.get_caller_id()+"I heard ")


def talker():
    rospy.init_node('yolo',anonymous=True)

    pub =rospy.Publisher('yolo_msg',String,queue_size=10)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/camera/depth/image_raw", Image, image_callback)
    rate =rospy.Rate(10)

    while not rospy.is_shutdown():
        hello_str ="hello world %s" %rospy.get_time()

        rospy.loginfo(hello_str)

        pub.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
