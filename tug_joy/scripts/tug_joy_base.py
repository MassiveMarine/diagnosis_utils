#!/usr/bin/env python

import rospy
from tug_joy_controller_mappings import *
from tug_joy_constants import *


class CallbackBase:
    def __init__(self, name, actuators, callback=None, cb_filtering=CB_FILTERING_NONE):
        self.name = name
        self.actuators = actuators
        self.cb_filtering = cb_filtering

        self.callback = callback if callback else self.callback_base

    def callback_base(self, values_dict):
        raise ValueError('Callback has to be overloaded')

    def __str__(self):
        return " Callback '" + str(self.name) + "' Actuators: " + str(self.actuators)

    def __repr__(self):
        return str(self)


class Manager:
    """
    The manager knows all actuators and executes there callbacks if one is set.
    """

    class __Manager:
        """
        Singleton pattern for manager class.
        """
        def __init__(self, rate, cb_at_break_once, cb_at_break_continuous, cb_at_exit):
            self.actuators = dict()
            self.callbacks = []
            self.rate = rospy.Rate(rate)
            self.cb_at_break_once = cb_at_break_once
            self.cb_at_break_once_already_called = False
            self.cb_at_break_continuous = cb_at_break_continuous
            self.cb_at_exit = cb_at_exit

    inst = None

    def __init__(self, contoller_name=CONTROLLER.CONTROLLER_NAME_DEFAULT, rate=20,
                 cb_at_break_once=None, cb_at_break_continuous=None, cb_at_exit=None):

        if not Manager.inst:
            Manager.inst = Manager.__Manager(rate, cb_at_break_once,
                                             cb_at_break_continuous, cb_at_exit)
            self.init_controller(contoller_name)

    @staticmethod
    def init_controller(contoller_name):
        """
        Load mapping between joy messages and general actuator names. With this
        different controllers are possible, because each button or axis is
        mapped to a defined general actuator.
        IF A NEW CONTROLLER IS ADDED, DON'T FORGET TO ADD IT HERE!!
        :param contoller_name: Defines which controller mapping should be used
        """
        if contoller_name == CONTROLLER.CONTROLLER_NAME_PS3:
            Manager.inst.actuators.update(PS3Mapping.mapping)
        elif contoller_name == CONTROLLER.CONTROLLER_NAME_LOGITECH_RUMBLE_PAD_2:
            Manager.inst.actuators.update(LogitechRumblePad2Mapping.mapping)
        elif contoller_name == CONTROLLER.CONTROLLER_NAME_XBOX360:
            Manager.inst.actuators.update(XBox360Mapping.mapping)
        elif contoller_name == CONTROLLER.CONTROLLER_NAME_DEFAULT:
            Manager.inst.actuators.update(DefaultMapping.mapping)
        else:
            raise ValueError("Controller name unknown. Maybe you forgot to add it to 'init_sticks' in 'tug_joy_base' if it is a new controller")

    def run(self, joy_subscriber):
        """
        Main loop of the controller manager. This loop iterates over all
        actuators  and executed the registered callback function if one is set.
        At the end of each loop, it checks if there is a new mapping, which
        should be loaded.
        :param joy_subscriber: rospy.Subscriber object is necessary to check if
               it exists and react if connection is lost.
        """
        # waiting for publisher subscriber connection
        while not rospy.is_shutdown() and joy_subscriber.get_num_connections() < 1:
            Manager.inst.rate.sleep()

        if rospy.is_shutdown():
            return

        while not rospy.is_shutdown():
            if joy_subscriber.get_num_connections() > 0:
                Manager.inst.cb_at_break_once_already_called = False

                for cb in Manager.inst.callbacks:
                    if cb.cb_filtering == CB_FILTERING_NONE:
                        cb.callback(dict((k, v.value) for k, v in Manager.inst.actuators.iteritems() if k in cb.actuators))
                        continue

                    for key, actuator in Manager.inst.actuators.iteritems():
                        if key not in cb.actuators or actuator.filtering != cb.cb_filtering:
                            continue

                        cb.callback(dict((k, v.value) for k, v in Manager.inst.actuators.iteritems() if k in cb.actuators))

                for actuator in Manager.inst.actuators.itervalues():
                    if actuator:
                        actuator.filtering = CB_FILTERING_NONE

            else:
                if not Manager.inst.cb_at_break_once_already_called and Manager.inst.cb_at_break_once:
                    Manager.inst.cb_at_break_once()
                    Manager.inst.cb_at_break_once_already_called = True
                if Manager.inst.cb_at_break_continuous:
                    Manager.inst.cb_at_break_continuous()

            Manager.inst.rate.sleep()

        if Manager.inst.cb_at_break_continuous:
            Manager.inst.cb_at_exit()

    @staticmethod
    def add_callback(cb):
        if cb not in Manager.inst.callbacks:

            for k, v in Manager.inst.actuators.iteritems():
                if k in cb.actuators and v.filtering == CB_FILTERING_DISABLED and cb.cb_filtering != CB_FILTERING_NONE:
                    rospy.logwarn("'" + cb.name + "' should be filterd but this is only possible if all actuators are buttons. Filter will be disabled for this callback!")
                    cb.cb_filtering = CB_FILTERING_NONE

            [v.set_used(True) for k, v in Manager.inst.actuators.iteritems() if k in cb.actuators]

            Manager.inst.callbacks.append(cb)
        else:
            rospy.logwarn("'" + cb.name + "' already in list!")

    @staticmethod
    def remove_callback(callback_fct):
        if callback_fct in Manager.inst.callbacks:
            Manager.inst.callbacks.remove(callback_fct)
            [v.set_used(False) for k, v in Manager.inst.actuators.iteritems() if k in callback_fct.actuators]
        else:
            rospy.logwarn("'" + callback_fct.name + "' NOT in list!")

    @staticmethod
    def joy_callback(msg):
        """
        Callback which is called if a new joy message is published by the joy node.
        It updates all values of each known actuator.
        :param msg: ROS subscriber message
        """
        for key, actuator in Manager.inst.actuators.iteritems():
            if not actuator:
                continue

            try:
                    actuator.set_value(msg)
            except:
                rospy.logerr("[" + actuator.name + "] set_value error!")
