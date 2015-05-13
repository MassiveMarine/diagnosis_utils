#!/usr/bin/env python

import rospy
from tug_joy_controller_mappings import *
from tug_joy_actuators import VirtualStickOf4
from tug_joy_actuators import VirtualStickOf2
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

    class __Manager:
        def __init__(self, contoller_name, rate, cb_at_break_once, cb_at_break_continuous, cb_at_exit):
            self.actuators = dict()
            self.rate = rospy.Rate(rate)
            self.new_mapping = []
            self.cb_at_break_once = cb_at_break_once
            self.cb_at_break_once_already_called = False
            self.cb_at_break_continuous = cb_at_break_continuous
            self.cb_at_exit = cb_at_exit

    instance = None

    def __init__(self, contoller_name=CONTROLLER.CONTROLLER_NAME_DEFAULT, rate=20,
                 cb_at_break_once=None, cb_at_break_continuous=None, cb_at_exit=None):

        if not Manager.instance:
            Manager.instance = Manager.__Manager(contoller_name, rate, cb_at_break_once,
                                                 cb_at_break_continuous, cb_at_exit)
            self.init_sticks(contoller_name)

    @staticmethod
    def init_sticks(contoller_name):
        if contoller_name == CONTROLLER.CONTROLLER_NAME_PS3:
            Manager.instance.actuators.update(PS3Mapping.mapping)
        elif contoller_name == CONTROLLER.CONTROLLER_NAME_LOGITECH_RUMBLE_PAD_2:
            Manager.instance.actuators.update(LogitechRumblePad2Mapping.mapping)
        elif contoller_name == CONTROLLER.CONTROLLER_NAME_XBOX360:
            Manager.instance.actuators.update(XBox360Mapping.mapping)
        else:
            Manager.instance.actuators.update(DefaultMapping.mapping)

    def run(self, joy_sub):
        # waiting for publisher subscriber connection
        while not rospy.is_shutdown() and joy_sub.get_num_connections() < 1:
            Manager.instance.rate.sleep()

        if rospy.is_shutdown():
            return

        while not rospy.is_shutdown():
            if joy_sub.get_num_connections() > 0:
                Manager.instance.cb_at_break_once_already_called = False
                for name, actuator in Manager.instance.actuators.iteritems():
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
                if len(Manager.instance.new_mapping) > 0:
                    self.set_function_mapping(Manager.instance.new_mapping, True)
                    Manager.instance.new_mapping = []
            else:
                if not Manager.instance.cb_at_break_once_already_called and Manager.instance.cb_at_break_once:
                    Manager.instance.cb_at_break_once()
                    Manager.instance.cb_at_break_once_already_called = True
                if Manager.instance.cb_at_break_continuous:
                    Manager.instance.cb_at_break_continuous()

            Manager.instance.rate.sleep()

        if Manager.instance.cb_at_break_continuous:
            Manager.instance.cb_at_exit()

    @staticmethod
    def set_function_mapping(new_mappings, force_mapping_change=False):
        if force_mapping_change:
            for new_mapping in new_mappings:
                if not get_correct_name(new_mapping.actuator_name, new_mapping.cb_filter_option) in Manager.instance.actuators.iterkeys():
                    rospy.logwarn("[" + str(new_mapping.actuator_name) + "] Actuator not found!")
                    continue

                if not Manager.instance.actuators[get_correct_name(new_mapping.actuator_name, new_mapping.cb_filter_option)]:
                    rospy.logwarn("[" + str(new_mapping.actuator_name) + "] No actuator mapped!")
                    continue

                Manager.instance.actuators[get_correct_name(new_mapping.actuator_name, new_mapping.cb_filter_option)].set_cb(new_mapping.callback_fct, new_mapping.cb_filter_option)
        else:
            for new_mapping in new_mappings:
                Manager.instance.new_mapping.append(new_mapping)

    @staticmethod
    def add_virtual_stick_of_4(up_name, right_name, down_name, left_name, callback_fct, name, invert_axes=[1.0, 1.0]):

        if not all(key in Manager.instance.actuators.keys() for key in [up_name, right_name, down_name, left_name]):
            rospy.logwarn("[" + name + "] one or more necessary actuator(s) not found!")
            return

        if not Manager.instance.actuators[up_name] or \
                not Manager.instance.actuators[right_name] or \
                not Manager.instance.actuators[down_name] or \
                not Manager.instance.actuators[left_name]:
            rospy.logwarn("[" + name + "] one or more necessary actuator(s) not mapped!")
            return

        Manager.instance.actuators.update({name: VirtualStickOf4(Manager.instance.actuators[up_name],
                                                                 Manager.instance.actuators[right_name],
                                                                 Manager.instance.actuators[down_name],
                                                                 Manager.instance.actuators[left_name],
                                                                 invert_axes, name, callback_fct)})

    @staticmethod
    def add_virtual_stick_of_2(horizontal_name, vertical_name, callback_fct, name):

        if not all(key in Manager.instance.actuators.keys() for key in [horizontal_name, vertical_name]):
            rospy.logwarn("[" + name + "] one or more necessary actuator(s) not found!")
            return

        if not Manager.instance.actuators[horizontal_name] or not Manager.instance.actuators[vertical_name]:
            rospy.logwarn("[" + name + "] one or more necessary actuator(s) not mapped!")
            return

        Manager.instance.actuators.update({name: VirtualStickOf2(Manager.instance.actuators[horizontal_name],
                                                                 Manager.instance.actuators[vertical_name],
                                                                 name, callback_fct)})

    @staticmethod
    def joy_callback(msg):
        for key, actuator in Manager.instance.actuators.iteritems():
            if not actuator:
                continue

            try:
                if actuator.__class__.__name__ in ['Button', 'Axis', 'Stick', 'VirtualButton', 'VirtualAxis']:
                    actuator.set_value(msg)
            except:
                rospy.logerr("[" + actuator.name + "] set_value error!")

        for key, actuator in Manager.instance.actuators.iteritems():
            if not actuator:
                continue

            try:
                if actuator.__class__.__name__ in ['VirtualStickOf4', 'VirtualStickOf2']:
                    actuator.set_value(msg)
            except:
                rospy.logerr("[" + actuator.name + "] set_value error!")