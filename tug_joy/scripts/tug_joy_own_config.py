#!/usr/bin/env python

"""tug_joy_own_config is used to define all needed callbacks and mappings,
which are needed for the current environment. There are also some predefined
callbacks and mappings."""

import rospy
from tug_joy_constants import *
from tug_joy_base import Mapping
from tug_joy_base import Manager

########################################################################################################################
#                                                 PREDEFINED MAPPINGS                                                  #
########################################################################################################################


def startup_config():
    """ This is the startup mapping. It will be loaded as first mapping.
    :return: Array of Mapping objects, that define the Mapping of actuators at
            startup. """
    return [Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, activate_setup_1, CB_FILTERING_PRESS),
            Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE),
            Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, activate_setup_2, CB_FILTERING_PRESS),
            Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, deactivate_setup_2, CB_FILTERING_RELEASE)]


########################################################################################################################
#                                                     OWN MAPPINGS                                                     #
# def example_config():                                                                                                #
#     return [Mapping(BUTTONS.FUNCTION_RIGHT, activate_setup_1, CB_FILTERING_PRESS),                                   #
#             Mapping(BUTTONS.FUNCTION_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE)]                                #
#                                                                                                                      #
########################################################################################################################
""" If there are mappings which are used several times in the code, they can be
defined in this section. """

########################################################################################################################
#                                                    OWN CALLBACKS                                                     #
# def example_cb(actuator):                                                                                            #
#     rospy.logdebug(str(actuator))                                                                                    #
#     # do what ever you want                                                                                          #
#                                                                                                                      #
########################################################################################################################
""" All callbacks are defined in this section. They can also be used to change
the current mapping """


def activate_setup_1(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_RIGHT, stick_cb)])


def deactivate_setup_1(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_RIGHT, None)], True)


def activate_setup_2(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_LEFT, stick_cb)])


def deactivate_setup_2(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_LEFT, None)], True)


def stick_cb(actuator):
    rospy.loginfo(str(actuator))


def axis_cb(actuator):
    rospy.loginfo(str(actuator))


########################################################################################################################
#                                                 PREDEFINED CALLBACKS                                                 #
########################################################################################################################

def manager_start_cb():
    """ This callback is called once at start time. It is used to load the
    initial callback mapping 'startup_config'. It can be used to set the
    system, robot, or whatever to a defined state. """
    rospy.logdebug('manager_start_cb')
    Manager().set_function_mapping(startup_config())


def manager_break_once_cb():
    """ This callback is called once if no joy-node-publisher-connection is
    available any more. If connection is lost, it is called once before
    'manager_break_continuous_cb()'. It can be used to set the system, robot,
    or whatever to a defined state. """
    rospy.logdebug('manager_break_once_cb')


def manager_break_continuous_cb():
    """ This callback is called continuous if no joy-node-publisher-connection
    is available any more. If connection is lost, it is called after
    'manager_break_once_cb()'. It can be used to set the system, robot, or
    whatever to a defined state. """
    rospy.logdebug('manager_break_continuous_cb')


def manager_exit_cb():
    """ This callback is called before this node will be killed. It can be used
    to set the system, robot, or whatever to a defined state. """
    rospy.logdebug('manager_exit_cb')

########################################################################################################################
#                                                 OWN DEFINED ACTUATORS                                                #
#                                                                                                                      #
# Manager().add_virtual_stick_of_4(AXIS.CROSS_1_AXIS_UP, AXIS.CROSS_1_AXIS_RIGHT,                                      #
#                                  AXIS.CROSS_1_AXIS_DOWN, AXIS.CROSS_1_AXIS_LEFT,                                     #
#                                  example_cb, 'ex_stick_1', [1.0, 1.0])                                               #
#                                                                                                                      #
# Manager().add_virtual_stick_of_2(AXIS.CROSS_2_AXIS_HORIZONTAL, AXIS.CROSS_2_AXIS_VERTICAL, example_cb, 'ex_stick_2') #
#                                                                                                                      #
########################################################################################################################


def special_actuators():
    """Define new actuators if you need one. This actuators are virtual, so
    they do not exist in real and are computed by using already defined
    actuators. An Virtual stick can be created out of 4 actuators or 2
    actuators for example. """
    pass

