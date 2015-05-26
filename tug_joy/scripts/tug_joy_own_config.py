#!/usr/bin/env python

"""
tug_joy_own_config is used to define all needed callbacks, which are needed for the current environment. There are also
some predefined callbacks.
"""

import rospy
from tug_joy_constants import *
from tug_joy_base import Manager
from tug_joy_base import Callback
from tug_joy_std_callbacks import *

########################################################################################################################
#                                                    OWN CALLBACKS                                                     #
# def example_cb_fct(values_dict):                                                                                     #
#     pass                                                                                                             #
#                                                                                                                      #
# example_cb_object = Callback('Example', [...used actuator...], example_cb_fct, ...filter option...)                  #
#                                                                                                                      #
########################################################################################################################
"""
All callback functions are defined in this section. Do not forget to create the Callback-object. The callback fct can
also be used to add or remove callbacks of the manager.
"""

cmd_vel_obj = CmdVel(actuator_linear_x=AXIS.STICK_AXIS_LEFT_VERTICAL,
                     actuator_linear_y=AXIS.STICK_AXIS_LEFT_HORIZONTAL,
                     actuator_angular_z=AXIS.STICK_AXIS_RIGHT_HORIZONTAL,
                     namespace='cmd_vel_1/',
                     publishing_topic='/cmd_vel')

cmd_vel = []
cmd_vel.append(Callback('cmd_vel_1', cmd_vel_obj.used_actuators, cmd_vel_obj.callback))
cmd_vel.append(Callback('cmd_vel_1_lin+', BUTTONS.CROSS_1_BUTTON_UP, cmd_vel_obj.increase_linear_speed_cb, CB_FILTERING_PRESS))
cmd_vel.append(Callback('cmd_vel_1_lin-', BUTTONS.CROSS_1_BUTTON_DOWN, cmd_vel_obj.decrease_linear_speed_cb, CB_FILTERING_PRESS))
cmd_vel.append(Callback('cmd_vel_1_ang+', BUTTONS.CROSS_1_BUTTON_LEFT, cmd_vel_obj.increase_angular_speed_cb, CB_FILTERING_PRESS))
cmd_vel.append(Callback('cmd_vel_1_ang-', BUTTONS.CROSS_1_BUTTON_RIGHT, cmd_vel_obj.decrease_angular_speed_cb, CB_FILTERING_PRESS))


def enable_disable_cmd_vel_cb(values_dict):
    if values_dict[BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT]:
        Manager().add_callback_list(cmd_vel)
    else:
        Manager().remove_callback_list(cmd_vel)

enable_cmd_vel = Callback('StickLeftOnCB', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], enable_disable_cmd_vel_cb, CB_FILTERING_PRESS)
disable_cmd_vel = Callback('StickLeftOnCB', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], enable_disable_cmd_vel_cb, CB_FILTERING_RELEASE)


# def stick_left_on_cb(values_dict):
#     Manager().add_callback(stick_left)
#
# stick_left_on = Callback('StickLeftOnCB', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], stick_left_on_cb, CB_FILTERING_PRESS)
#
#
# def stick_left_off_cb(values_dict):
#     Manager().remove_callback(stick_left)
#
# stick_left_off = Callback('StickLeftOffCB', [BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT], stick_left_off_cb, CB_FILTERING_RELEASE)
#
#
# def stick_right_on_cb(values_dict):
#     Manager().add_callback(stick_right)
#
# stick_right_on = Callback('StickRightOnCB', [BUTTONS.SHOULDER_BUTTON_UPPER_LEFT], stick_right_on_cb, CB_FILTERING_PRESS)
#
#
# def stick_right_off_cb(values_dict):
#     Manager().remove_callback(stick_right)
#
# stick_right_off = Callback('StickRightOffCB', [BUTTONS.SHOULDER_BUTTON_UPPER_LEFT], stick_right_off_cb, CB_FILTERING_RELEASE)
#
#
# def stick_1_cb(values_dict):
#     print values_dict
#
# stick_left = Callback('StichLeftCB', [AXIS.STICK_AXIS_LEFT_HORIZONTAL, AXIS.STICK_AXIS_LEFT_VERTICAL], stick_1_cb)
# stick_right = Callback('StichRightCB', [AXIS.STICK_AXIS_RIGHT_HORIZONTAL, AXIS.STICK_AXIS_RIGHT_VERTICAL], stick_1_cb)


########################################################################################################################
#                                                 PREDEFINED CALLBACKS                                                 #
########################################################################################################################

def manager_start_cb():
    """
    This callback is called once at start time. It is used to load the initial callbacks. It can also be used to set
    the system, robot, or whatever to a defined state.
    """
    rospy.logdebug('manager_start_cb')
    # Manager().add_callback(stick_left_on)
    # Manager().add_callback(stick_left_off)
    # Manager().add_callback(stick_right_on)
    # Manager().add_callback(stick_right_off)
    # Manager().add_callback_list(cmd_vel)
    Manager().add_callback(enable_cmd_vel)
    Manager().add_callback(disable_cmd_vel)


def manager_break_once_cb():
    """
    This callback is called once if no joy-node-publisher-connection is
    available any more. If connection is lost, it is called once before
    'manager_break_continuous_cb()'. It can be used to set the system, robot,
    or whatever to a defined state.
    """
    rospy.logdebug('manager_break_once_cb')


def manager_break_continuous_cb():
    """
    This callback is called continuous if no joy-node-publisher-connection
    is available any more. If connection is lost, it is called after
    'manager_break_once_cb()'. It can be used to set the system, robot, or
    whatever to a defined state.
    """
    rospy.logdebug('manager_break_continuous_cb')


def manager_exit_cb():
    """ This callback is called before this node will be killed. It can be used
    to set the system, robot, or whatever to a defined state. """
    rospy.logdebug('manager_exit_cb')
