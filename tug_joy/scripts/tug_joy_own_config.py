#!/usr/bin/env python

# basics
import rospy
from tug_joy_constants import *
from tug_joy_base import Mapping
from tug_joy_base import Manager

global manager
########################################################################################################################
#                                                     OWN MAPPINGS                                                     #
# def example_config():                                                                                                #
#     return [Mapping(BUTTONS.FUNCTION_RIGHT, activate_setup_1, CB_FILTERING_PRESS),                                   #
#             Mapping(BUTTONS.FUNCTION_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE)]                                #
#                                                                                                                      #
########################################################################################################################


def startup_config():
    return [Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, activate_setup_1, CB_FILTERING_PRESS),
            Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE),
            Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, activate_setup_2, CB_FILTERING_PRESS),
            Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, deactivate_setup_2, CB_FILTERING_RELEASE)]


########################################################################################################################
#                                                    OWN CALLBACKS                                                     #
# def example_cb(actuator):                                                                                            #
#     rospy.logdebug(str(actuator))                                                                                    #
#     # do what ever you want                                                                                          #
#                                                                                                                      #
########################################################################################################################


def activate_setup_1(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_RIGHT, stick_cb)])


def deactivate_setup_1(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_RIGHT, None)])


def activate_setup_2(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_LEFT, stick_cb)])


def deactivate_setup_2(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_LEFT, None)])


def stick_cb(actuator):
    rospy.loginfo(str(actuator))


########################################################################################################################
#                                                 PREDEFINED CALLBACKS                                                 #
########################################################################################################################

def manager_break_once_cb():
    rospy.logdebug('manager_break_once_cb')


def manager_break_continuous_cb():
    rospy.logdebug('manager_break_continuous_cb')


def manager_exit_cb():
    rospy.logdebug('manager_exit_cb')

