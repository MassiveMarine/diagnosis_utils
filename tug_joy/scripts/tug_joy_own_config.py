#!/usr/bin/env python

"""
tug_joy_own_config is used to define all needed callbacks and mappings,
which are needed for the current environment. There are also some predefined
callbacks and mappings.
"""

import rospy
from tug_joy_constants import *
from tug_joy_base import Mapping
from tug_joy_base import Manager

########################################################################################################################
#                                                 PREDEFINED MAPPINGS                                                  #
########################################################################################################################


def startup_config():
    """
    This is the startup mapping. It will be loaded as first mapping.
    :return: Array of Mapping objects, that define the Mapping of actuators at
             startup.
    """
    # return [Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, activate_setup_1, CB_FILTERING_PRESS),
            # Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE),
            # Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, activate_setup_2, CB_FILTERING_PRESS),
            # Mapping(BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, deactivate_setup_2, CB_FILTERING_RELEASE)]
    return [Mapping('first_group', cmd_vel_1_.callback),
            Mapping(BUTTONS.CROSS_1_BUTTON_UP, cmd_vel_1_.increase_linear_speed, CB_FILTERING_PRESS),
            Mapping(BUTTONS.CROSS_1_BUTTON_DOWN, cmd_vel_1_.decrease_linear_speed, CB_FILTERING_PRESS),
            Mapping(BUTTONS.CROSS_1_BUTTON_LEFT, cmd_vel_1_.increase_angular_speed, CB_FILTERING_PRESS),
            Mapping(BUTTONS.CROSS_1_BUTTON_RIGHT, cmd_vel_1_.decrease_angular_speed, CB_FILTERING_PRESS)]


########################################################################################################################
#                                                     OWN MAPPINGS                                                     #
# def example_config():                                                                                                #
#     return [Mapping(BUTTONS.FUNCTION_RIGHT, activate_setup_1, CB_FILTERING_PRESS),                                   #
#             Mapping(BUTTONS.FUNCTION_LEFT, deactivate_setup_1, CB_FILTERING_RELEASE)]                                #
#                                                                                                                      #
########################################################################################################################
"""
If there are mappings which are used several times in the code, they can be
defined in this section.
"""

########################################################################################################################
#                                                    OWN CALLBACKS                                                     #
# def example_cb(actuator):                                                                                            #
#     rospy.logdebug(str(actuator))                                                                                    #
#     # do what ever you want                                                                                          #
#                                                                                                                      #
########################################################################################################################
"""
All callbacks are defined in this section. They can also be used to change
the current mapping.
"""


def activate_setup_1(actuator):
    rospy.logdebug(str(actuator))
    Manager().set_function_mapping([Mapping(STICK.STICK_RIGHT, cmd_vel_1_.callback)])


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


def group_cb(actuator):
    rospy.loginfo(str(actuator))





########################################################################################################################
#                                                 PREDEFINED CALLBACKS                                                 #
########################################################################################################################

def manager_start_cb():
    """
    This callback is called once at start time. It is used to load the
    initial callback mapping 'startup_config'. It can be used to set the
    system, robot, or whatever to a defined state.
    """
    rospy.logdebug('manager_start_cb')
    cmd_vel_1_.load_parameters('cmd_vel_1/', '/cmd_vel', AXIS.STICK_AXIS_LEFT_VERTICAL,
                               AXIS.STICK_AXIS_LEFT_HORIZONTAL, AXIS.STICK_AXIS_RIGHT_HORIZONTAL)
    Manager().set_function_mapping(startup_config())


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
    """
    Define new actuators if you need one. This actuators are virtual, so
    they do not exist in real and are computed by using already defined
    actuators. An Virtual stick can be created out of 4 actuators or 2
    actuators for example.
    """
    Manager.add_actuator_group('first_group',
                               [AXIS.STICK_AXIS_LEFT_HORIZONTAL, AXIS.STICK_AXIS_LEFT_VERTICAL, AXIS.STICK_AXIS_RIGHT_HORIZONTAL])
    pass


class CmdVel:
    from geometry_msgs.msg import Twist

    def __init__(self):

        self.namespace = None
        self.max_tv_ = None
        self.max_rv_ = None
        self.min_tv_ = None
        self.min_rv_ = None
        self.basic_tv_ = None
        self.basic_rv_ = None

        self.actuator_linear_x = None
        self.actuator_linear_y = None
        self.actuator_angular_z = None

        self.cmd_vel_pub_ = None

    def load_parameters(self, namespace, publishing_topic, actuator_linear_x, actuator_linear_y, actuator_angular_z):
        self.namespace = namespace

        self.max_tv_ = rospy.get_param('~' + self.namespace + 'MaxTransVel', 1.0)
        self.max_rv_ = rospy.get_param('~' + self.namespace + 'MinTransVel', 1.0)
        self.min_tv_ = rospy.get_param('~' + self.namespace + 'MaxRotVel', 1.0)
        self.min_rv_ = rospy.get_param('~' + self.namespace + 'MinRotVel', 1.0)
        self.basic_tv_ = rospy.get_param('~' + self.namespace + 'InitTransVel', 1.0)
        self.basic_rv_ = rospy.get_param('~' + self.namespace + 'InitRotVel', 1.0)

        self.cmd_vel_pub_ = rospy.Publisher(publishing_topic, self.Twist, queue_size=1)
        self.actuator_linear_x = actuator_linear_x
        self.actuator_linear_y = actuator_linear_y
        self.actuator_angular_z = actuator_angular_z

    def increase_linear_speed(self, actuator):
        self.basic_tv_ += self.max_tv_ / 10
        if self.basic_tv_ > self.max_tv_:
            self.basic_tv_ = self.max_tv_

    def decrease_linear_speed(self, actuator):
        self.basic_tv_ -= self.max_tv_ / 10
        if self.basic_tv_ < self.min_tv_:
            self.basic_tv_ = self.min_tv_

    def increase_angular_speed(self, actuator):
        self.basic_rv_ += self.max_rv_ / 10
        if self.basic_rv_ > self.max_rv_:
            self.basic_rv_ = self.max_rv_

    def decrease_angular_speed(self, actuator):
        self.basic_rv_ -= self.max_rv_ / 10
        if self.basic_rv_ < self.min_rv_:
            self.basic_tv_ = self.min_rv_

    def callback(self, actuator):
        new_twist = self.Twist()
        try:
            if self.actuator_linear_x:
                new_twist.linear.x = self.basic_tv_ * actuator.get_value(self.actuator_linear_x)
            if self.actuator_linear_y:
                new_twist.linear.y = self.basic_tv_ * actuator.get_value(self.actuator_linear_y)
            if self.actuator_angular_z:
                new_twist.angular.z = self.basic_rv_ * actuator.get_value(self.actuator_angular_z)
            self.cmd_vel_pub_.publish(new_twist)
        except ValueError as error:
            rospy.logerr(error)

cmd_vel_1_ = CmdVel()


