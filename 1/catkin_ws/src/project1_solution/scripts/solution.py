#!/usr/bin/env python
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

#author: SkookumAsFrig


def callback(data):
    sumab = data.a + data.b
    rospy.loginfo("data received is " + str(data.a) + " and " + str(data.b) +
                  ", and sum is " + str(sumab))
    pub.publish(Int16(sumab))


def subber():
    rospy.Subscriber('two_ints', TwoInts, callback)


def pubber():
    global pub
    pub = rospy.Publisher('sum', Int16, queue_size=10)
    rospy.init_node('pubber', anonymous=True)


if __name__ == '__main__':
    try:
        pubber()
        subber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
