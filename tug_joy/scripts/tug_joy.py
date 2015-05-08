#!/usr/bin/env python

# basics
import rospy
from sensor_msgs.msg import Joy
from tug_joy_base import Mapping
from tug_joy_base import Manager
from tug_joy_constants import *


def button_cb(stick):
    # pass
    rospy.loginfo('button' + str(stick))


def stick1_cb(stick):
    # pass
    rospy.loginfo('stick1' + str(stick))


def stick2_cb(stick):
    rospy.loginfo('stick2' + str(stick))


def axis_cb(axis):
    rospy.loginfo('axis_cb' + str(axis))


def activate_setup_1(button):
    rospy.loginfo('activate setup 1 by' + str(button))
    manager.set_function_mapping([Mapping(STICK.STICK_RIGHT, stick1_cb)])


def deactivate_setup_1(button):
    rospy.loginfo('deactivate setup 1 by' + str(button))
    manager.set_function_mapping([Mapping(STICK.STICK_RIGHT, None)])


def activate_setup_2(button):
    rospy.loginfo('activate setup 2 by' + str(button))
    manager.set_function_mapping([Mapping(STICK.STICK_LEFT, stick2_cb)])


def deactivate_setup_2(button):
    rospy.loginfo('deactivate setup 2 by' + str(button))
    manager.set_function_mapping([Mapping(STICK.STICK_LEFT, None)])


def manager_break_once_cb():
    rospy.logdebug('manager_break_once_cb')


def manager_break_continuous_cb():
    rospy.logdebug('manager_break_continuous_cb')


def manager_exit_cb():
    rospy.logdebug('manager_exit_cb')


manager = None
if __name__ == "__main__":
    rospy.init_node('tug_joy', anonymous=True)

    try:
        manager = Manager(CONTROLLER_NAME_PS3, 20, manager_break_once_cb, manager_break_continuous_cb, manager_exit_cb)
        # manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, activate_setup_1, CB_FILTERING_PRESS)], True)
        # manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE)], True)
        #
        # manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, activate_setup_2, CB_FILTERING_PRESS)], True)
        # manager.set_function_mapping([Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, deactivate_setup_2, CB_FILTERING_RELEASE)], True)

        # manager.set_function_mapping([Mapping(BUTTONS.CROSS_1_BUTTON_LEFT, button_cb)], True)
        # manager.set_function_mapping([Mapping(BUTTONS.CROSS_1_BUTTON_UP, button_cb)], True)
        # manager.set_function_mapping([Mapping(BUTTONS.CROSS_1_BUTTON_RIGHT, button_cb)], True)
        # manager.set_function_mapping([Mapping(BUTTONS.CROSS_1_BUTTON_DOWN, button_cb)], True)

        manager.set_function_mapping([Mapping(AXIS.CROSS_2_AXIS_HORIZONTAL, axis_cb)], True)
        manager.set_function_mapping([Mapping(AXIS.CROSS_2_AXIS_VERTICAL, axis_cb)], True)

        # manager.add_virtual_stick(AXIS.CROSS_1_AXIS_UP,
        #                           AXIS.CROSS_1_AXIS_RIGHT,
        #                           AXIS.CROSS_1_AXIS_DOWN,
        #                           AXIS.CROSS_1_AXIS_LEFT,
        #                           stick1_cb, 'vstick1', [-1.0, -1.0])

        # manager.add_virtual_stick(BUTTONS.CROSS_1_BUTTON_UP,
        #                           BUTTONS.CROSS_1_BUTTON_RIGHT,
        #                           BUTTONS.CROSS_1_BUTTON_DOWN,
        #                           BUTTONS.CROSS_1_BUTTON_LEFT,
        #                           stick1_cb, 'avstick1', [1.0, 1.0])

        joy_sub = rospy.Subscriber("joy", Joy, manager.joy_callback)
        manager.run(joy_sub)

    except KeyboardInterrupt:
        manager.stop()
        pass
    except rospy.ROSInterruptException:
        manager.stop()
        pass
