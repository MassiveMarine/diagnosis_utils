#!/usr/bin/env python

# basics
import rospy
from sensor_msgs.msg import Joy
# from tug_joy_base import Mapping
from tug_joy_base import Manager
from tug_joy_constants import *

from tug_joy_own_config import *


# def button_cb(stick):
#     # rospy.loginfo(str(stick))
#     pass
#
# def stick_cb(stick):
#     # rospy.loginfo(str(stick))
#     pass
#
#
# def axis_cb(axis):
#     # rospy.loginfo(str(axis))
#     pass
#
# def stick1_cb(stick):
#     # pass
#     rospy.loginfo('stick1' + str(stick))
#
#
# def stick2_cb(stick):
#     rospy.loginfo('stick2' + str(stick))
#
#
#
#
#
# def activate_setup_1(button):
#     rospy.loginfo('activate setup 1 by' + str(button))
#     manager.set_function_mapping([Mapping(STICK.STICK_RIGHT, stick1_cb)])
#
#
# def deactivate_setup_1(button):
#     rospy.loginfo('deactivate setup 1 by' + str(button))
#     manager.set_function_mapping([Mapping(STICK.STICK_RIGHT, None)])
#
#
# def activate_setup_2(button):
#     rospy.loginfo('activate setup 2 by' + str(button))
#     manager.set_function_mapping([Mapping(STICK.STICK_LEFT, stick2_cb)])
#
#
# def deactivate_setup_2(button):
#     rospy.loginfo('deactivate setup 2 by' + str(button))
#     manager.set_function_mapping([Mapping(STICK.STICK_LEFT, None)])
#
#
# def manager_break_once_cb():
#     rospy.logdebug('manager_break_once_cb')
#
#
# def manager_break_continuous_cb():
#     rospy.logdebug('manager_break_continuous_cb')
#
#
# def manager_exit_cb():
#     rospy.logdebug('manager_exit_cb')
#
# def stick_cb(stick):
#     rospy.loginfo(str(stick))
#     # pass



manager = None
if __name__ == "__main__":
    rospy.init_node('tug_joy', anonymous=True)

    try:
        manager = Manager(CONTROLLER.CONTROLLER_NAME_DEFAULT, 20,
                          manager_break_once_cb,
                          manager_break_continuous_cb,
                          manager_exit_cb)
        #
        manager.set_function_mapping(startup_config())

        # manager.set_function_mapping([Mapping(BUTTONS.CROSS_1_BUTTON_LEFT, button_cb),
        #                               Mapping(BUTTONS.CROSS_1_BUTTON_RIGHT, button_cb),
        #                               Mapping(BUTTONS.CROSS_1_BUTTON_UP, button_cb),
        #                               Mapping(BUTTONS.CROSS_1_BUTTON_DOWN, button_cb),
        #
        #                               Mapping(BUTTONS.CROSS_2_BUTTON_LEFT, button_cb),
        #                               Mapping(BUTTONS.CROSS_2_BUTTON_RIGHT, button_cb),
        #                               Mapping(BUTTONS.CROSS_2_BUTTON_UP, button_cb),
        #                               Mapping(BUTTONS.CROSS_2_BUTTON_DOWN, button_cb),
        #
        #                               Mapping(BUTTONS.FUNCTION_CENTER, button_cb),
        #                               Mapping(BUTTONS.FUNCTION_LEFT, button_cb),
        #                               Mapping(BUTTONS.FUNCTION_RIGHT, button_cb),
        #
        #                               Mapping(BUTTONS.STICK_BUTTON_LEFT, button_cb),
        #                               Mapping(BUTTONS.STICK_BUTTON_RIGHT, button_cb),
        #
        #                               Mapping(BUTTONS.SHOULDER_BUTTON_LOWER_LEFT, button_cb),
        #                               Mapping(BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT, button_cb),
        #                               Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, button_cb),
        #                               Mapping(BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT, button_cb),
        #                               Mapping(BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT, button_cb),
        #                               Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, button_cb)], True)
        #
        # manager.set_function_mapping([Mapping(BUTTONS.CROSS_1_BUTTON_LEFT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.CROSS_1_BUTTON_RIGHT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.CROSS_1_BUTTON_UP, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.CROSS_1_BUTTON_DOWN, button_cb, CB_FILTERING_PRESS),
        #
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_LEFT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_RIGHT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_UP, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_DOWN, button_cb, CB_FILTERING_PRESS),
        #
        #                       Mapping(BUTTONS.FUNCTION_CENTER, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.FUNCTION_LEFT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.FUNCTION_RIGHT, button_cb, CB_FILTERING_PRESS),
        #
        #                       Mapping(BUTTONS.STICK_BUTTON_LEFT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.STICK_BUTTON_RIGHT, button_cb, CB_FILTERING_PRESS),
        #
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_LOWER_LEFT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT, button_cb, CB_FILTERING_PRESS),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, button_cb, CB_FILTERING_PRESS)], True)
        #
        # manager.set_function_mapping([Mapping(BUTTONS.CROSS_1_BUTTON_LEFT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.CROSS_1_BUTTON_RIGHT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.CROSS_1_BUTTON_UP, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.CROSS_1_BUTTON_DOWN, button_cb, CB_FILTERING_RELEASE),
        #
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_LEFT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_RIGHT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_UP, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.CROSS_2_BUTTON_DOWN, button_cb, CB_FILTERING_RELEASE),
        #
        #                       Mapping(BUTTONS.FUNCTION_CENTER, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.FUNCTION_LEFT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.FUNCTION_RIGHT, button_cb, CB_FILTERING_RELEASE),
        #
        #                       Mapping(BUTTONS.STICK_BUTTON_LEFT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.STICK_BUTTON_RIGHT, button_cb, CB_FILTERING_RELEASE),
        #
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_LOWER_LEFT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT, button_cb, CB_FILTERING_RELEASE),
        #                       Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, button_cb, CB_FILTERING_RELEASE)], True)

        # manager.set_function_mapping([Mapping(AXIS.CROSS_1_AXIS_LEFT, axis_cb),
        #                             Mapping(AXIS.CROSS_1_AXIS_UP, axis_cb),
        #                             Mapping(AXIS.CROSS_1_AXIS_DOWN, axis_cb),
        #                             Mapping(AXIS.CROSS_1_AXIS_RIGHT, axis_cb),
        #                             Mapping(AXIS.CROSS_1_AXIS_HORIZONTAL, axis_cb),
        #                             Mapping(AXIS.CROSS_1_AXIS_VERTICAL, axis_cb),
        #
        #                             Mapping(AXIS.CROSS_2_AXIS_LEFT, axis_cb),
        #                             Mapping(AXIS.CROSS_2_AXIS_UP, axis_cb),
        #                             Mapping(AXIS.CROSS_2_AXIS_DOWN, axis_cb),
        #                             Mapping(AXIS.CROSS_2_AXIS_RIGHT, axis_cb),
        #                             Mapping(AXIS.CROSS_2_AXIS_HORIZONTAL, axis_cb),
        #                             Mapping(AXIS.CROSS_2_AXIS_VERTICAL, axis_cb),
        #
        #                             Mapping(AXIS.STICK_AXIS_LEFT_HORIZONTAL, axis_cb),
        #                             Mapping(AXIS.STICK_AXIS_LEFT_VERTICAL, axis_cb),
        #                             Mapping(AXIS.STICK_AXIS_RIGHT_HORIZONTAL, axis_cb),
        #                             Mapping(AXIS.STICK_AXIS_RIGHT_VERTICAL, axis_cb),
        #
        #                             Mapping(AXIS.SHOULDER_AXIS_LOWER_LEFT, axis_cb),
        #                             Mapping(AXIS.SHOULDER_AXIS_UPPER_LEFT, axis_cb),
        #                             Mapping(AXIS.SHOULDER_AXIS_MIDDLE_LEFT, axis_cb),
        #                             Mapping(AXIS.SHOULDER_AXIS_LOWER_RIGHT, axis_cb),
        #                             Mapping(AXIS.SHOULDER_AXIS_UPPER_RIGHT, axis_cb),
        #                             Mapping(AXIS.SHOULDER_AXIS_MIDDLE_RIGHT, axis_cb),
        #
        #                             Mapping(AXIS.ACCELERATION_X, axis_cb),
        #                             Mapping(AXIS.ACCELERATION_Y, axis_cb),
        #                             Mapping(AXIS.ACCELERATION_Z, axis_cb),
        #
        #                             Mapping(AXIS.GYROSCOPE_ROLL, axis_cb),
        #                             Mapping(AXIS.GYROSCOPE_PITCH, axis_cb),
        #                             Mapping(AXIS.GYROSCOPE_YAW, axis_cb),
        #
        #                             Mapping(STICK.STICK_LEFT, stick_cb),
        #                             Mapping(STICK.STICK_RIGHT, stick_cb)
        #                             ], True)



        # manager.add_virtual_stick(AXIS.CROSS_1_AXIS_UP,
        # AXIS.CROSS_1_AXIS_RIGHT,
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
