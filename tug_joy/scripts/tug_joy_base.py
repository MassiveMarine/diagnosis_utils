#!/usr/bin/env python

import rospy
from tug_joy_controller_mappings import ButtonMapping
from tug_joy_controller_mappings import AxisMapping
from tug_joy_controller_mappings import StickMapping
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

    def __init__(self, contoller_name=CONTROLLER_NAME_DEFAULT, rate=20):
        self.actuators = dict()
        self.init_sticks(contoller_name)
        self.rate = rospy.Rate(rate)
        self.new_mapping = []

    def init_sticks(self, contoller_name):
        if contoller_name == CONTROLLER_NAME_PS3:
            self.actuators.update(ButtonMapping.ps3_to_general)
            self.actuators.update(AxisMapping.ps3_to_general)
            self.actuators.update(StickMapping.ps3_to_general)
        else:
            self.actuators = dict()

    def run(self):
        while not rospy.is_shutdown():
            try:
                for name, actuator in self.actuators.iteritems():
                    if not actuator or not actuator.callback_fct:
                        continue
                    if actuator.callback_filtering == CB_FILTERING_NONE:
                        actuator.callback_fct(actuator)
                    elif actuator.callback_filtering == actuator.change_since_last:
                        actuator.change_since_last = CB_FILTERING_NONE
                        actuator.callback_fct(actuator)

                # change to new mapping if a callback changed it
                if len(self.new_mapping) > 0:
                    self.set_function_mapping(self.new_mapping, True)
                    self.new_mapping = []
            except:
                rospy.logerr("[Manager] Error while executing callbacks")

            self.rate.sleep()

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

        self.actuators.update({name: VirtualStick(self.actuators[up_name],   self.actuators[right_name],
                                                  self.actuators[down_name], self.actuators[left_name],
                                                  invert_axes, name, callback_fct)})

    def joy_callback(self, msg):
        for key, actuator in self.actuators.iteritems():
            if actuator:
                actuator.set_value(msg)