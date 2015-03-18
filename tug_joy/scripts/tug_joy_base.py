#!/usr/bin/env python

# basics
from time import struct_time
import rospy
from sensor_msgs.msg import Joy

# JOYSTICK_AXIS_STICK_0_HORZ = 0
# JOYSTICK_AXIS_STICK_0_VERT = 1
#
# JOYSTICK_AXIS_STICK_1_HORZ = 2
# JOYSTICK_AXIS_STICK_1_VERT = 3
#
# mapping = {0: JOYSTICK_AXIS_STICK_0_HORZ,
# :"abc"}


class Stick:
    def __init__(self, horizontal_id, vertical_id):
        self.horizontal_val = 0.0
        self.horizontal_id = horizontal_id
        self.vertical_val = 0.0
        self.vertical_id = vertical_id
        self.callback_fct = None

    def stick_cb(self, msg):
        try:
            self.horizontal_val = msg.axes[self.horizontal_id]
            self.vertical_val = msg.axes[self.vertical_id]
        except:
            print "stick id not found"

        if self.callback_fct:
            self.callback_fct(self)

    def __str__(self):
        return "h: " + str(self.horizontal_val) + " v: " + str(self.vertical_val)

    def __repr__(self):
        return str(self)


class VirtualStick:
    def __init__(self, source_type, up_id, right_id, down_id, left_id, invert_axes=[False, False]):
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
            self.horizontal_val *= 1.0 if not self.invert_axes[0] else -1.0
            self.vertical_val = source[self.up_id] - source[self.down_id]
            self.vertical_val *= 1.0 if not self.invert_axes[1] else -1.0
        except:
            print "button id not found"

    def __str__(self):
        return "h: " + str(self.horizontal_val) + " v: " + str(self.vertical_val)

    def __repr__(self):
        return str(self)


class Manager:
    def __init__(self, contoller_name):
        self.sticks = []
        self.axes = []
        self.buttons = []
        self.init_sticks(contoller_name)

    def init_sticks(self, contoller_name):
        if contoller_name == 'ps3':
            self.sticks.append(Stick(0, 1))
            self.sticks.append(Stick(2, 3))
        elif contoller_name == 'default':
            self.sticks.append(Stick(0, 1))
            self.sticks.append(Stick(3, 2))
            self.sticks.append(VirtualStick('buttons', 1, 3, 2, 0))
        else:
            print "contoller name not found"

    def joy_callback(self, msg):
        for stick in self.sticks:
            stick.stick_cb(msg)
            # try:
            #     stick.horizontal_val = msg.axes[stick.horizontal_id]
            #     stick.vertical_val = msg.axes[stick.vertical_id]
            # except:
            #     print "stick id not found"

        # print self.sticks


# if __name__ == "__main__":
#     rospy.init_node('tug_joy_base', anonymous=True)
#
#     try:
#         print 'Hallo'
#     except KeyboardInterrupt:
#         pass
#     except rospy.ROSInterruptException:
#         pass