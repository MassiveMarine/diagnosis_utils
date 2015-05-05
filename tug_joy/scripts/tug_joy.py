#!/usr/bin/env python

# basics
import rospy
from sensor_msgs.msg import Joy
import tug_joy_base
from tug_joy_constants import *


def axes1_cb(axis):
    print axis


def stick1_cb(stick):
    print stick


def button1_cb(button):
    print "1"


def button2_cb(button):
    print "2"


def button3_cb(button):
    print button


def activate_setup_1(button):
    print 'activate setup 1'
    manager.set_function_mapping({BUTTONS.FUNCTION_LEFT: [activate_setup_2, CB_FILTERING_PRESS]})


def activate_setup_2(button):
    print 'activate setup 2'
    manager.set_function_mapping({BUTTONS.FUNCTION_LEFT: [activate_setup_1, CB_FILTERING_PRESS]})

manager = None
if __name__ == "__main__":
    rospy.init_node('tug_joy', anonymous=True)

    try:
        manager = tug_joy_base.Manager(CONTROLLER_NAME_PS3)
        manager.set_function_mapping({BUTTONS.FUNCTION_LEFT: [activate_setup_1, CB_FILTERING_PRESS]}, True)
        # manager.set_function_mapping({BUTTONS.CROSS_2_BUTTON_DOWN: [button1_cb, CB_FILTERING_NONE]}, True)
        # manager.set_function_mapping({BUTTONS.CROSS_2_BUTTON_RIGHT: [button2_cb, CB_FILTERING_NONE]}, True)

        # manager.set_function_mapping({AXIS.CROSS_1_AXIS_LEFT: [axes1_cb, CB_FILTERING_NONE]})
        # manager.set_function_mapping({STICK.STICK_LEFT: [stick1_cb, CB_FILTERING_NONE]})

        # manager.add_virtual_stick(AXIS.CROSS_1_AXIS_UP,
        #                           AXIS.CROSS_1_AXIS_RIGHT,
        #                           AXIS.CROSS_1_AXIS_DOWN,
        #                           AXIS.CROSS_1_AXIS_LEFT,
        #                           stick1_cb, 'vstick1', [-1.0, -1.0])

        rospy.Subscriber("joy", Joy, manager.joy_callback)
        manager.run()

    except KeyboardInterrupt:
        manager.stop()
        pass
    except rospy.ROSInterruptException:
        manager.stop()
        pass
