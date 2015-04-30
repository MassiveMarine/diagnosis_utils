#!/usr/bin/env python

# basics
import rospy
from sensor_msgs.msg import Joy
import tug_joy_base


def axes1_cb(axis):
    print axis

def stick1_cb(stick):
    print stick

def button1_cb(button):
    print button


if __name__ == "__main__":
    rospy.init_node('tug_joy', anonymous=True)

    try:
        manager = tug_joy_base.Manager()
        # print manager.sticks
        # manager.sticks[2].callback_fct = stick1_cb
        manager.axes[0].callback_fct = axes1_cb

        rospy.Subscriber("joy", Joy, manager.joy_callback)

        rospy.spin()

    except KeyboardInterrupt:
        manager.stop()
        pass
    except rospy.ROSInterruptException:
        manager.stop()
        pass
