#!/usr/bin/env python

import rospy
from tug_joy_controller_mappings import *
# from tug_joy_actuators import VirtualStickOf4
# from tug_joy_actuators import VirtualStickOf2
# from tug_joy_actuators import Group
from tug_joy_constants import *


# class Mapping:
#     """
#     Structure for defining mapping between callback and general actuator name.
#     """
#
#     def __init__(self, actuator_name, callback_fct, cb_filter_option=CB_FILTERING_NONE):
#         self.actuator_name = actuator_name
#         self.callback_fct = callback_fct
#         self.cb_filter_option = cb_filter_option
#
#     def __str__(self):
#         return 'Mapping: ' + str(self.actuator_name) + ' -> ' + str(self.callback_fct.__name__) + ' with ' + str(
#             self.cb_filter_option) + '-filter'
#
#     def __repr__(self):
#         return str(self)


class Manager:
    """
    The manager knows all actuators and executes there callbacks if one is set.
    """

    class __Manager:
        """
        Singleton pattern for manager class.
        """
        def __init__(self, contoller_name, rate, cb_at_break_once, cb_at_break_continuous, cb_at_exit):
            self.actuators = dict()
            self.callbacks = []
            self.rate = rospy.Rate(rate)
            self.new_mapping = []
            self.cb_at_break_once = cb_at_break_once
            self.cb_at_break_once_already_called = False
            self.cb_at_break_continuous = cb_at_break_continuous
            self.cb_at_exit = cb_at_exit

    inst = None

    def __init__(self, contoller_name=CONTROLLER.CONTROLLER_NAME_DEFAULT, rate=20,
                 cb_at_break_once=None, cb_at_break_continuous=None, cb_at_exit=None):

        if not Manager.inst:
            Manager.inst = Manager.__Manager(contoller_name, rate, cb_at_break_once,
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
                    if cb.callback_filtering == CB_FILTERING_NONE:
                        cb.callback(dict((k, v) for k, v in Manager.inst.actuators.iteritems() if k in cb.actuators))
                        continue

                    for key, actuator in Manager.inst.actuators.iteritems():
                        if key not in cb.actuators or actuator.change_since_last != cb.callback_filtering:
                            continue

                        cb.callback(dict((k, v) for k, v in Manager.inst.actuators.iteritems() if k in cb.actuators))

                for actuator in Manager.inst.actuators.itervalues():
                    if actuator:
                        actuator.change_since_last = CB_FILTERING_NONE

                        # print 'min. eine aenderung'
                    # else:
                    #     print 'keine aenderung'


                    # print cb.actuators
                    # cb.callback(dict(
                    #     (key, value) for key, value in Manager.inst.actuators.iteritems() if key in cb.actuators))

                # for name, actuator in Manager.instance.actuators.iteritems():
                #     try:
                #
                #         if not actuator or not actuator.callback_fct:
                #             continue
                #         if actuator.callback_filtering == CB_FILTERING_NONE:
                #             actuator.callback_fct(actuator)
                #         elif actuator.callback_filtering == actuator.change_since_last:
                #             actuator.change_since_last = CB_FILTERING_NONE
                #             actuator.callback_fct(actuator)
                #
                #     except:
                #         rospy.logerr("[Manager] Error while executing callbacks")

                # change to new mapping if a callback changed it
                if len(Manager.inst.new_mapping) > 0:
                    self.set_function_mapping(Manager.inst.new_mapping, True)
                    Manager.inst.new_mapping = []
            else:
                if not Manager.inst.cb_at_break_once_already_called and Manager.inst.cb_at_break_once:
                    Manager.inst.cb_at_break_once()
                    Manager.inst.cb_at_break_once_already_called = True
                if Manager.inst.cb_at_break_continuous:
                    Manager.inst.cb_at_break_continuous()

            Manager.inst.rate.sleep()

        if Manager.inst.cb_at_break_continuous:
            Manager.inst.cb_at_exit()

    # @staticmethod
    # def set_function_mapping(new_mappings, force_mapping_change=False):
    #     """
    #     Set new callback functions for actuators.
    #     :param new_mappings: Array of Mapping-objects
    #     :param force_mapping_change: If the mapping should be loaded
    #            immediately set it so 'True'. Otherwise changes are applied at
    #            the end of each loop of the main loop.
    #     :return:
    #     """
    #     if force_mapping_change:
    #         for new_mapping in new_mappings:
    #             if not get_correct_name(new_mapping.actuator_name, new_mapping.cb_filter_option) in Manager.inst.actuators.iterkeys():
    #                 rospy.logwarn("[" + str(new_mapping.actuator_name) + "] Actuator not found!")
    #                 continue
    #
    #             if not Manager.inst.actuators[get_correct_name(new_mapping.actuator_name, new_mapping.cb_filter_option)]:
    #                 rospy.logwarn("[" + str(new_mapping.actuator_name) + "] No actuator mapped!")
    #                 continue
    #
    #             Manager.inst.actuators[get_correct_name(new_mapping.actuator_name, new_mapping.cb_filter_option)].set_cb(new_mapping.callback_fct, new_mapping.cb_filter_option)
    #     else:
    #         for new_mapping in new_mappings:
    #             Manager.inst.new_mapping.append(new_mapping)

    # @staticmethod
    # def add_virtual_stick_of_4(up_name, right_name, down_name, left_name, callback_fct, name, invert_axes=[1.0, 1.0]):
    #     """
    #     This function can be used to add a virtual stick out of four inputs (up, down, left and right).
    #     The inputs can be axes or buttons and can also be mixed up.
    #     :param up_name: Name of actuator, which should be used for up.
    #     :param right_name: Name of actuator, which should be used for right.
    #     :param down_name: Name of actuator, which should be used for down.
    #     :param left_name: Name of actuator, which should be used for left.
    #     :param callback_fct: Function pointer to set a callback function.
    #     :param name: Name of the new (Virtual)-Actuator
    #     :param invert_axes: Can be set to invert (and scale) the horizontal and
    #            vertical behaviour of the stick. (1.0 => no inverting, -1.0 => inverting)
    #     :return: True if everything is ok, otherwise False.
    #     """
    #
    #     if not all(key in Manager.inst.actuators.keys() for key in [up_name, right_name, down_name, left_name]):
    #         rospy.logwarn("[" + name + "] one or more necessary actuator(s) not found!")
    #         return False
    #
    #     if not Manager.inst.actuators[up_name] or \
    #             not Manager.inst.actuators[right_name] or \
    #             not Manager.inst.actuators[down_name] or \
    #             not Manager.inst.actuators[left_name]:
    #         rospy.logwarn("[" + name + "] one or more necessary actuator(s) not mapped!")
    #         return False
    #
    #     Manager.inst.actuators.update({name: VirtualStickOf4(Manager.inst.actuators[up_name],
    #                                                              Manager.inst.actuators[right_name],
    #                                                              Manager.inst.actuators[down_name],
    #                                                              Manager.inst.actuators[left_name],
    #                                                              invert_axes, name, callback_fct)})
    #     return True

    # @staticmethod
    # def add_virtual_stick_of_2(horizontal_name, vertical_name, callback_fct, name, invert_axes=[1.0, 1.0]):
    #     """
    #     This function can be used to add a virtual stick out of two inputs (horizontal and vertical).
    #     The inputs should only be axes, because buttons makes no sense, but it should work too.
    #     :param horizontal_name: Name of actuator, which should be used for horizontal.
    #     :param vertical_name: Name of actuator, which should be used for vertical.
    #     :param callback_fct: Function pointer to set a callback function.
    #     :param name: Name of the new (Virtual)-Actuator
    #     :param invert_axes: Can be set to invert (and scale) the horizontal and
    #            vertical behaviour of the stick. (1.0 => no inverting, -1.0 => inverting)
    #     :return: True if everything is ok, otherwise False.
    #     """
    #
    #     if not all(key in Manager.inst.actuators.keys() for key in [horizontal_name, vertical_name]):
    #         rospy.logwarn("[" + name + "] one or more necessary actuator(s) not found!")
    #         return False
    #
    #     if not Manager.inst.actuators[horizontal_name] or not Manager.inst.actuators[vertical_name]:
    #         rospy.logwarn("[" + name + "] one or more necessary actuator(s) not mapped!")
    #         return False
    #
    #     Manager.inst.actuators.update({name: VirtualStickOf2(Manager.inst.actuators[horizontal_name],
    #                                                              Manager.inst.actuators[vertical_name],
    #                                                              invert_axes, name, callback_fct)})
    #     return True

    # @staticmethod
    # def add_actuator_group(name, actuator_names, callback_fct=None):
    #     if not all(key in Manager.inst.actuators.keys() for key in actuator_names):
    #         rospy.logwarn("[" + name + "] one or more necessary actuator(s) not found!")
    #         return False
    #
    #     # if not Manager.instance.actuators[horizontal_name] or not Manager.instance.actuators[vertical_name]:
    #     #     rospy.logwarn("[" + name + "] one or more necessary actuator(s) not mapped!")
    #     #     return False
    #
    #     Manager.inst.actuators.update({name: Group([Manager.inst.actuators[actuator] for actuator in actuator_names], name, callback_fct)})

    @staticmethod
    def add_callback(callback_fct):
        if callback_fct not in Manager.inst.callbacks:
            [v.set_used(True) for k, v in Manager.inst.actuators.iteritems() if k in callback_fct.actuators]

            Manager.inst.callbacks.append(callback_fct)
        else:
            rospy.logwarn("'" + callback_fct.name + "' already in list!")

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
