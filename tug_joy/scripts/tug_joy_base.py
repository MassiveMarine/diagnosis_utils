#!/usr/bin/env python

# basics
from ldb import valid_attr_name

import rospy
import threading
import mappings




class Actuator:
    def __init__(self, name='unknown', callback_fct=None):
        self.name = name
        self.callback_fct = callback_fct

    def __str__(self):
        return str(self.name)


class Button(Actuator):
    def __init__(self, button_id, name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.button_val = 0.0
        self.button_id = button_id

    def button_cb(self, msg):
        try:
            self.button_val = msg.buttons[self.button_id]
        except:
            print "[" + Actuator.__str__(self) + "] button id '", self.button_id, "' not found"

        # if self.callback_fct:
        #     self.callback_fct(self)

    def __str__(self):
        return 'B ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.button_val)

    def __repr__(self):
        return str(self)


class Axis(Actuator):
    def __init__(self, axis_id, name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.axis_val = 0.0
        self.axis_id = axis_id

    def axis_cb(self, msg):
        try:
            self.axis_val = msg.axes[self.axis_id]
        except:
            print "[" + Actuator.__str__(self) + "] axis id '", self.axis_id, "' not found"

        # if self.callback_fct:
        #     self.callback_fct(self)

    def __str__(self):
        return 'A ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.axis_val)

    def __repr__(self):
        return str(self)


class Stick(Actuator):
    def __init__(self, horizontal_id, vertical_id, name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.horizontal_val = 0.0
        self.horizontal_id = horizontal_id
        self.vertical_val = 0.0
        self.vertical_id = vertical_id

    def stick_cb(self, msg):
        try:
            self.horizontal_val = msg.axes[self.horizontal_id]
            self.vertical_val = msg.axes[self.vertical_id]
        except:
            print "[" + Actuator.__str__(self) + "] stick id '", self.horizontal_id, "' or '", self.vertical_id, "' not found"

        # if self.callback_fct:
            # self.callback_fct(self)

    def __str__(self):
        return 'S ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)


class VirtualStick(Actuator):
    def __init__(self, source_type, up_id, right_id, down_id, left_id, invert_axes=[1.0, 1.0], name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.source_type = source_type
        self.invert_axes = invert_axes
        self.horizontal_val = 0.0
        self.vertical_val = 0.0
        self.up_id = up_id
        self.right_id = right_id
        self.down_id = down_id
        self.left_id = left_id

    def stick_cb(self, msg):
        if self.source_type == "buttons":
            source = msg.buttons
        elif self.source_type == "axes":
            source = msg.axes
        else:
            print "source type '", self.source_type, "' not found"

        try:
            self.horizontal_val = source[self.left_id] - source[self.right_id]
            self.horizontal_val *= self.invert_axes[0]
            self.vertical_val = source[self.up_id] - source[self.down_id]
            self.vertical_val *= self.invert_axes[1]
        except:
            print "[" + Actuator.__str__(self) + "] button id not found"

        # if self.callback_fct:
        #     self.callback_fct(self)

    def __str__(self):
        return 'S ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)


class Generator(threading.Thread):
    def __init__(self, inputs, rate):
        threading.Thread.__init__(self)
        self.thread_event = thread_event
        self.rate = rospy.Rate(rate)  # 10hz
        self.inputs = inputs

    def run(self):
        while not rospy.is_shutdown() or self.thread_event.is_set():
            for name, actuator in self.inputs.iteritems():
                if actuator.callback_fct:
                    actuator.callback_fct(actuator)
            self.rate.sleep()


threads = []
thread_event = threading.Event()


class Manager:

    def __init__(self, contoller_name='default'):
        self.sticks = []
        self.axes = []
        self.buttons = dict()
        self.init_sticks(contoller_name)

        # self.thread_event = threading.Event()
        # self.threads = []

    def init_sticks(self, contoller_name):
        # self.buttons['cross_right_button_left'] = Button(15)
        # self.buttons['cross_right_button_up'] = Button(12)
        # self.buttons['cross_right_button_down'] = Button(14)
        # self.buttons['cross_right_button_right'] = Button(13)
        self.buttons = mappings.ps3_to_general_mapping_buttons

        # if contoller_name == 'ps3':
        #     self.buttons.append(Button(14, 'cross'))
        # elif contoller_name == 'default':
        #     self.sticks.append(Stick(0, 1, 'left'))
        #     self.sticks.append(Stick(3, 2, 'right'))
        #     self.sticks.append(VirtualStick('buttons', 1, 3, 2, 0, [1.0, 1.0], 'right'))
        # else:
        #     print "contoller name not found"
        # self.axes.append(Axis(17, 'circle'))
        # self.axes.append(Axis(18, 'cross'))
        thread1 = Generator(self.buttons, 20)
        # thread1.start()
        # threads.append(Generator(thread_event, self.axes, 20))
        #
        # for thread in threads:
        #     thread.start()

        threads.append(thread1)

    def stop(self):
        thread_event.clear()
        for thread in threads:
            thread.join()

    def run(self):
        thread_event.clear()
        for thread in threads:
            thread.start()

    def set_function_mapping(self, mapping):
        for key, func_ptr in mapping.iteritems():
            self.buttons[key].callback_fct = func_ptr

    def joy_callback(self, msg):
        # for stick in self.sticks:
        #     stick.stick_cb(msg)
        for key, button in self.buttons.iteritems():
            button.button_cb(msg)
        # for axis in self.axes:
        #     axis.axis_cb(msg)

        # print self.axes






            # self.sticks.append(Stick(0, 1, 'left'))
            # self.sticks.append(Stick(2, 3, 'right'))
            # self.sticks.append(VirtualStick('axes', 8, 9, 10, 11, [-0.5, -0.5], 'left'))
            # self.sticks.append(VirtualStick('axes', 16, 17, 18, 19, [-0.5, -0.5], 'right'))
            # self.axes.append(Axis(0, 'stick_l_hori'))
            # self.axes.append(Axis(1, 'stick_l_vert'))
            # self.axes.append(Axis(2, 'stick_r_hori'))
            # self.axes.append(Axis(3, 'stick_r_vert'))
            # self.axes.append(Axis(8, 'cross_up'))
            # self.axes.append(Axis(9, 'cross_right'))
            # self.axes.append(Axis(10, 'cross_down'))
            # self.axes.append(Axis(11, 'cross_left'))
            # self.axes.append(Axis(12, 'shoulder_ll'))
            # self.axes.append(Axis(13, 'shoulder_lr'))
            # self.axes.append(Axis(14, 'shoulder_ul'))
            # self.axes.append(Axis(15, 'shoulder_ur'))
            # self.axes.append(Axis(16, 'triangle'))
            # self.axes.append(Axis(17, 'circle'))
            # self.axes.append(Axis(18, 'cross'))
            # self.axes.append(Axis(19, 'square'))
            # self.axes.append(Axis(24, 'acc_x'))
            # self.axes.append(Axis(23, 'acc_y'))
            # self.axes.append(Axis(25, 'acc_z'))
            # self.axes.append(Axis(26, 'gyro_yaw'))
            #
            # self.axes.append(Axis(16, 'triangle'))
            # self.axes.append(Axis(17, 'circle'))
            # self.axes.append(Axis(18, 'cross'))
            # self.axes.append(Axis(19, 'square'))
            # self.buttons.append(Button(0, 'select'))
            # self.buttons.append(Button(1, 'stick_left'))
            # self.buttons.append(Button(2, 'stick_right'))
            # self.buttons.append(Button(3, 'start'))
            # self.buttons.append(Button(4, 'cross_up'))
            # self.buttons.append(Button(5, 'cross_right'))
            # self.buttons.append(Button(6, 'cross_down'))
            # self.buttons.append(Button(7, 'cross_left'))
            # self.buttons.append(Button(8, 'shoulder_ll'))
            # self.buttons.append(Button(9, 'shoulder_lr'))
            # self.buttons.append(Button(10, 'shoulder_ul'))
            # self.buttons.append(Button(11, 'shoulder_ur'))
            # self.buttons.append(Button(12, 'triangle'))
            # self.buttons.append(Button(13, 'circle'))
            # self.buttons.append(Button(14, 'cross'))
            # self.buttons.append(Button(15, 'square'))
            # self.buttons.append(Button(16, 'ps'))