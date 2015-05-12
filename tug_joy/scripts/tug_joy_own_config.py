#!/usr/bin/env python

# basics
import rospy
from tug_joy_constants import *
from tug_joy_base import Mapping

global manager
########################################################################################################################
#                                                     OWN MAPPINGS                                                     #
########################################################################################################################


def empty_config():
    return []


def startup_config():
    return [Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, activate_setup_1, CB_FILTERING_PRESS),
            Mapping(STICK.STICK_RIGHT, stick_cb)]


def setup_1_config():
    return [Mapping(BUTTONS.FUNCTION_LEFT, stick_cb)]


def setup_2_config():
    return [Mapping(BUTTONS.FUNCTION_LEFT, stick_cb)]

########################################################################################################################
#                                                    OWN CALLBACKS                                                     #
########################################################################################################################

def activate_setup_1(input):
    pass

def stick_cb(stick):
    rospy.loginfo(str(stick))
    # pass


def axis_cb(axis):
    # rospy.loginfo(str(axis))
    pass

########################################################################################################################
#                                                 PREDEFINED CALLBACKS                                                 #
########################################################################################################################

def manager_break_once_cb():
    rospy.logdebug('manager_break_once_cb')


def manager_break_continuous_cb():
    rospy.logdebug('manager_break_continuous_cb')


def manager_exit_cb():
    rospy.logdebug('manager_exit_cb')

