#!/usr/bin/env python

# basics
import rospy
from sensor_msgs.msg import Joy
import tug_joy_base


def stick1_cb(stick):
    print stick


if __name__ == "__main__":
    rospy.init_node('tug_joy_base', anonymous=True)

    try:
        manager = tug_joy_base.Manager('default')
        manager.sticks[0].callback_fct = stick1_cb

        rospy.Subscriber("joy", Joy, manager.joy_callback)

        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass