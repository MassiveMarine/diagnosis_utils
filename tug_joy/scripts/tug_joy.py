#!/usr/bin/env python

# basics
import rospy
from sensor_msgs.msg import Joy
from tug_joy_base import Mapping
from tug_joy_base import Manager
from tug_joy_constants import *


def stick1_cb(stick):
    print 'stick1', stick


def stick2_cb(stick):
    print 'stick2', stick


def activate_setup_1(button):
    print 'activate setup 1 by', button
    manager.set_function_mapping([Mapping(STICK.STICK_RIGHT, stick1_cb)])


def deactivate_setup_1(button):
    print 'deactivate setup 1 by', button
    manager.set_function_mapping([Mapping(STICK.STICK_RIGHT, None)])


def activate_setup_2(button):
    print 'activate setup 2 by', button
    manager.set_function_mapping([Mapping(STICK.STICK_LEFT, stick2_cb)])


def deactivate_setup_2(button):
    print 'deactivate setup 2 by', button
    manager.set_function_mapping([Mapping(STICK.STICK_LEFT, None)])

manager = None
if __name__ == "__main__":
    rospy.init_node('tug_joy', anonymous=True)

    try:
        manager = Manager(CONTROLLER_NAME_PS3, 20)
        manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, activate_setup_1, CB_FILTERING_PRESS)], True)
        manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE)], True)

        manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, activate_setup_2, CB_FILTERING_PRESS)], True)
        manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, deactivate_setup_2, CB_FILTERING_RELEASE)], True)


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
