#!/usr/bin/env python

import rospy
from tug_joy_mappings import ButtonMapping
from tug_joy_mappings import AxisMapping
from tug_joy_mappings import StickMapping

from tug_joy_actuators import VirtualStick


from tug_joy_constants import *


class Manager:

    def __init__(self, contoller_name=CONTROLLER_NAME_DEFAULT, rate=20):
        # self.sticks = dict()
        # self.axes = dict()
        # self.buttons = dict()
        self.actuators = dict()
        self.init_sticks(contoller_name)
        self.rate = rospy.Rate(rate)
        self.new_mapping = dict()

    def init_sticks(self, contoller_name):
        if contoller_name == CONTROLLER_NAME_PS3:
            self.actuators.update(ButtonMapping.ps3_to_general)
            self.actuators.update(AxisMapping.ps3_to_general)
            self.actuators.update(StickMapping.ps3_to_general)
        else:
            # self.sticks = dict()
            # self.axes = dict()
            # self.buttons = dict()
            self.actuators = dict()

    def run(self):
        while not rospy.is_shutdown():
            # for name, actuator in self.buttons.iteritems():
            for name, actuator in self.actuators.iteritems():
                if not actuator or not actuator.callback_fct:
                    continue
                if actuator.callback_filtering == CB_FILTERING_NONE:
                    actuator.callback_fct(actuator)
                elif actuator.callback_filtering == actuator.change_since_last:
                    actuator.change_since_last = CB_FILTERING_NONE
                    actuator.callback_fct(actuator)

            # change to new mapping if a callback changed it
            if len(self.new_mapping.items()) > 0:
                self.set_function_mapping(self.new_mapping, True)
                self.new_mapping = dict()

            self.rate.sleep()

    def set_function_mapping(self, mapping, force_mapping_change=False):
        if force_mapping_change:
            for key, func_ptr in mapping.iteritems():
                # if key in self.buttons.keys():
                #     self.buttons[key].callback_fct = func_ptr[0]
                #     self.buttons[key].callback_filtering = func_ptr[1]
                if key in self.actuators.keys():
                    self.actuators[key].callback_fct = func_ptr[0]
                    self.actuators[key].callback_filtering = func_ptr[1]
        else:
            for key, func_ptr in mapping.iteritems():
                self.new_mapping[key] = func_ptr

    def add_virtual_stick(self, up_name, right_name, down_name, left_name, callback_fct, name, invert_axes=[ 1.0, 1.0]):
        # (self, source_type, invert_axes=[1.0, 1.0], name='unknown', callback_fct=None)
        self.actuators.update({name: VirtualStick(self.actuators[up_name], self.actuators[right_name], self.actuators[down_name],
                                                  self.actuators[left_name], invert_axes, name, callback_fct)})


    def joy_callback(self, msg):
        # for stick in self.sticks:
        #     stick.stick_cb(msg)
        # for key, button in self.buttons.iteritems():
        for key, actuator in self.actuators.iteritems():
            if actuator:
                actuator.callback(msg)
        # for axis in self.axes:
        #     axis.axis_cb(msg)