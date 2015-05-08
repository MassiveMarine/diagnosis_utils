#!/usr/bin/env python

import rospy
from tug_joy_controller_mappings import PS3Mapping
from tug_joy_controller_mappings import DefaultMapping
# from tug_joy_controller_mappings import StickMapping
from tug_joy_controller_mappings import get_correct_name

from tug_joy_actuators import VirtualStick


from tug_joy_constants import *


class Mapping:
    def __init__(self, actuator_name, callback_fct, cb_filter_option=CB_FILTERING_NONE):
        self.actuator_name = actuator_name
        self.callback_fct = callback_fct
        self.cb_filter_option = cb_filter_option

    def __str__(self):
        return 'Mapping: ' + str(self.actuator_name) + ' -> ' + str(self.callback_fct.__name__) + ' with ' + str(self.cb_filter_option) + '-filter'

    def __repr__(self):
        return str(self)


class Manager:

    def __init__(self, contoller_name=CONTROLLER_NAME_DEFAULT, rate=20, cb_at_break_once=None, cb_at_break_continuous=None, cb_at_exit=None):
        self.actuators = dict()
        self.init_sticks(contoller_name)
        self.rate = rospy.Rate(rate)
        self.new_mapping = []
        self.cb_at_break_once = cb_at_break_once
        self.cb_at_break_once_already_called = False
        self.cb_at_break_continuous = cb_at_break_continuous
        self.cb_at_exit = cb_at_exit

    def init_sticks(self, contoller_name):
        if contoller_name == CONTROLLER_NAME_PS3:
            self.actuators.update(PS3Mapping.mapping)
        else:
            self.actuators.update(DefaultMapping.mapping)

    def run(self, joy_sub):
        # waiting for publisher subscriber connection
        while not rospy.is_shutdown() and joy_sub.get_num_connections() < 1:
            self.rate.sleep()

        if rospy.is_shutdown():
            return

        while not rospy.is_shutdown():
            if joy_sub.get_num_connections() > 0:
                self.cb_at_break_once_already_called = False
                for name, actuator in self.actuators.iteritems():
                    try:

                        if not actuator or not actuator.callback_fct:
                            continue
                        if actuator.callback_filtering == CB_FILTERING_NONE:
                            actuator.callback_fct(actuator)
                        elif actuator.callback_filtering == actuator.change_since_last:
                            actuator.change_since_last = CB_FILTERING_NONE
                            actuator.callback_fct(actuator)

                    except:
                        rospy.logerr("[Manager] Error while executing callbacks")

                    # change to new mapping if a callback changed it
                    if len(self.new_mapping) > 0:
                        self.set_function_mapping(self.new_mapping, True)
                        self.new_mapping = []
            else:
                if not self.cb_at_break_once_already_called and self.cb_at_break_once:
                    self.cb_at_break_once()
                    self.cb_at_break_once_already_called = True
                if self.cb_at_break_continuous:
                    self.cb_at_break_continuous()

            self.rate.sleep()

        if self.cb_at_break_continuous:
            self.cb_at_exit()

    def set_function_mapping(self, new_mappings, force_mapping_change=False):
        if force_mapping_change:
            for new_mapping in new_mappings:
                self.actuators[get_correct_name(new_mapping.actuator_name, new_mapping.cb_filter_option)].set_cb(new_mapping.callback_fct, new_mapping.cb_filter_option)
        else:
            for new_mapping in new_mappings:
                self.new_mapping.append(new_mapping)

    def add_virtual_stick(self, up_name, right_name, down_name, left_name, callback_fct, name, invert_axes=[1.0, 1.0]):

        if not all(key in self.actuators.keys() for key in [up_name, right_name, down_name, left_name]):
            rospy.logwarn("[" + name + "] one or more necessary actuator(s) not found!")
            return

        if not self.actuators[up_name] or not self.actuators[right_name] or not self.actuators[down_name] or not self.actuators[left_name]:
            rospy.logwarn("[" + name + "] one or more necessary actuator(s) not mapped!")
            return

        self.actuators.update({name: VirtualStick(self.actuators[up_name],   self.actuators[right_name],
                                                  self.actuators[down_name], self.actuators[left_name],
                                                  invert_axes, name, callback_fct)})

    def joy_callback(self, msg):
        for key, actuator in self.actuators.iteritems():
            if not actuator:
                continue

            if actuator.__class__.__name__ in ['Button', 'Axis', 'Stick', 'VirtualButton', 'VirtualAxis']:
                actuator.set_value(msg)

        for key, actuator in self.actuators.iteritems():
            if not actuator:
                continue

            if actuator.__class__.__name__ in ['VirtualStick']:
                actuator.set_value(msg)