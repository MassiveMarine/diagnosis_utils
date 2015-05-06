#!/usr/bin/env python

from tug_joy_actuators import Button
from tug_joy_actuators import Axis
from tug_joy_actuators import Stick

from tug_joy_constants import *


def get_correct_name(actuator_name, filter_option):
    return actuator_name if filter_option == CB_FILTERING_NONE else (actuator_name, filter_option)


class ButtonMapping:

    ps3_to_general = {
        # Button mapping general
        BUTTONS.CROSS_1_BUTTON_LEFT:  Button(7, 'cross_1_button_left'),
        BUTTONS.CROSS_1_BUTTON_UP:    Button(4, 'cross_1_button_up'),
        BUTTONS.CROSS_1_BUTTON_DOWN:  Button(6, 'cross_1_button_down'),
        BUTTONS.CROSS_1_BUTTON_RIGHT: Button(5, 'cross_1_button_right'),

        BUTTONS.CROSS_2_BUTTON_LEFT:  Button(15, 'cross_2_button_left'),
        BUTTONS.CROSS_2_BUTTON_UP:    Button(12, 'cross_2_button_up'),
        BUTTONS.CROSS_2_BUTTON_DOWN:  Button(14, 'cross_2_button_down'),
        BUTTONS.CROSS_2_BUTTON_RIGHT: Button(13, 'cross_2_button_right'),

        BUTTONS.FUNCTION_LEFT:   Button(0,  'function_left'),
        BUTTONS.FUNCTION_RIGHT:  Button(3,  'function_right'),
        BUTTONS.FUNCTION_CENTER: Button(16, 'function_center'),

        BUTTONS.STICK_BUTTON_LEFT:  Button(1, 'stick_button_left'),
        BUTTONS.STICK_BUTTON_RIGHT: Button(2, 'stick_button_right'),

        BUTTONS.SHOULDER_BUTTON_LOWER_LEFT:   Button(8,  'shoulder_button_lower_left'),
        BUTTONS.SHOULDER_BUTTON_UPPER_LEFT:   Button(10, 'shoulder_button_upper_left'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT:  None,
        BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT:  Button(9,  'shoulder_button_lower_right'),
        BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT:  Button(11, 'shoulder_button_upper_right'),
        BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT: None,

        # Button mapping at pressing
        (BUTTONS.CROSS_1_BUTTON_LEFT, CB_FILTERING_PRESS):  Button(7, 'cross_1_button_left'),
        (BUTTONS.CROSS_1_BUTTON_UP, CB_FILTERING_PRESS):    Button(4, 'cross_1_button_up'),
        (BUTTONS.CROSS_1_BUTTON_DOWN, CB_FILTERING_PRESS):  Button(6, 'cross_1_button_down'),
        (BUTTONS.CROSS_1_BUTTON_RIGHT, CB_FILTERING_PRESS): Button(5, 'cross_1_button_right'),

        (BUTTONS.CROSS_2_BUTTON_LEFT, CB_FILTERING_PRESS):  Button(15, 'cross_2_button_left'),
        (BUTTONS.CROSS_2_BUTTON_UP, CB_FILTERING_PRESS):    Button(12, 'cross_2_button_up'),
        (BUTTONS.CROSS_2_BUTTON_DOWN, CB_FILTERING_PRESS):  Button(14, 'cross_2_button_down'),
        (BUTTONS.CROSS_2_BUTTON_RIGHT, CB_FILTERING_PRESS): Button(13, 'cross_2_button_right'),

        (BUTTONS.FUNCTION_LEFT, CB_FILTERING_PRESS):   Button(0,  'function_left'),
        (BUTTONS.FUNCTION_RIGHT, CB_FILTERING_PRESS):  Button(3,  'function_right'),
        (BUTTONS.FUNCTION_CENTER, CB_FILTERING_PRESS): Button(16, 'function_center'),

        (BUTTONS.STICK_BUTTON_LEFT, CB_FILTERING_PRESS):  Button(1, 'stick_button_left'),
        (BUTTONS.STICK_BUTTON_RIGHT, CB_FILTERING_PRESS): Button(2, 'stick_button_right'),

        (BUTTONS.SHOULDER_BUTTON_LOWER_LEFT, CB_FILTERING_PRESS):   Button(8,  'shoulder_button_lower_left'),
        (BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, CB_FILTERING_PRESS):   Button(10, 'shoulder_button_upper_left'),
        (BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT, CB_FILTERING_PRESS):  None,
        (BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT, CB_FILTERING_PRESS):  Button(9,  'shoulder_button_lower_right'),
        (BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, CB_FILTERING_PRESS):  Button(11, 'shoulder_button_upper_right'),
        (BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT, CB_FILTERING_PRESS): None,

        # Button mapping at releasing
        (BUTTONS.CROSS_1_BUTTON_LEFT, CB_FILTERING_RELEASE):  Button(7, 'cross_1_button_left'),
        (BUTTONS.CROSS_1_BUTTON_UP, CB_FILTERING_RELEASE):    Button(4, 'cross_1_button_up'),
        (BUTTONS.CROSS_1_BUTTON_DOWN, CB_FILTERING_RELEASE):  Button(6, 'cross_1_button_down'),
        (BUTTONS.CROSS_1_BUTTON_RIGHT, CB_FILTERING_RELEASE): Button(5, 'cross_1_button_right'),

        (BUTTONS.CROSS_2_BUTTON_LEFT, CB_FILTERING_RELEASE):  Button(15, 'cross_2_button_left'),
        (BUTTONS.CROSS_2_BUTTON_UP, CB_FILTERING_RELEASE):    Button(12, 'cross_2_button_up'),
        (BUTTONS.CROSS_2_BUTTON_DOWN, CB_FILTERING_RELEASE):  Button(14, 'cross_2_button_down'),
        (BUTTONS.CROSS_2_BUTTON_RIGHT, CB_FILTERING_RELEASE): Button(13, 'cross_2_button_right'),

        (BUTTONS.FUNCTION_LEFT, CB_FILTERING_RELEASE):   Button(0,  'function_left'),
        (BUTTONS.FUNCTION_RIGHT, CB_FILTERING_RELEASE):  Button(3,  'function_right'),
        (BUTTONS.FUNCTION_CENTER, CB_FILTERING_RELEASE): Button(16, 'function_center'),

        (BUTTONS.STICK_BUTTON_LEFT, CB_FILTERING_RELEASE):  Button(1, 'stick_button_left'),
        (BUTTONS.STICK_BUTTON_RIGHT, CB_FILTERING_RELEASE): Button(2, 'stick_button_right'),

        (BUTTONS.SHOULDER_BUTTON_LOWER_LEFT, CB_FILTERING_RELEASE):   Button(8,  'shoulder_button_lower_left'),
        (BUTTONS.SHOULDER_BUTTON_UPPER_LEFT, CB_FILTERING_RELEASE):   Button(10, 'shoulder_button_upper_left'),
        (BUTTONS.SHOULDER_BUTTON_MIDDLE_LEFT, CB_FILTERING_RELEASE):  None,
        (BUTTONS.SHOULDER_BUTTON_LOWER_RIGHT, CB_FILTERING_RELEASE):  Button(9,  'shoulder_button_lower_right'),
        (BUTTONS.SHOULDER_BUTTON_UPPER_RIGHT, CB_FILTERING_RELEASE):  Button(11, 'shoulder_button_upper_right'),
        (BUTTONS.SHOULDER_BUTTON_MIDDLE_RIGHT, CB_FILTERING_RELEASE): None
    }


class AxisMapping:
    ps3_to_general = {
        # Axis mapping
        AXIS.CROSS_1_AXIS_LEFT:  Axis(11, 'cross_1_axis_left'),
        AXIS.CROSS_1_AXIS_UP:    Axis(8,  'cross_1_axis_up'),
        AXIS.CROSS_1_AXIS_DOWN:  Axis(10, 'cross_1_axis_down'),
        AXIS.CROSS_1_AXIS_RIGHT: Axis(9,  'cross_1_axis_right'),

        AXIS.CROSS_2_AXIS_LEFT:  Axis(19, 'cross_2_axis_left'),
        AXIS.CROSS_2_AXIS_UP:    Axis(16, 'cross_2_axis_up'),
        AXIS.CROSS_2_AXIS_DOWN:  Axis(18, 'cross_2_axis_down'),
        AXIS.CROSS_2_AXIS_RIGHT: Axis(17, 'cross_2_axis_right'),

        AXIS.STICK_AXIS_LEFT_HORIZONTAL:  Axis(0, 'stick_axis_left_horizontal'),
        AXIS.STICK_AXIS_LEFT_VERTICAL:    Axis(1, 'stick_axis_left_vertical'),
        AXIS.STICK_AXIS_RIGHT_HORIZONTAL: Axis(2, 'stick_axis_right_horizontal'),
        AXIS.STICK_AXIS_RIGHT_VERTICAL:   Axis(3, 'stick_axis_right_vertical'),

        AXIS.SHOULDER_AXIS_LOWER_LEFT:   Axis(12, 'shoulder_axis_lower_left'),
        AXIS.SHOULDER_AXIS_UPPER_LEFT:   Axis(14, 'shoulder_axis_upper_left'),
        AXIS.SHOULDER_AXIS_MIDDLE_LEFT:  None,
        AXIS.SHOULDER_AXIS_LOWER_RIGHT:  Axis(13, 'shoulder_axis_lower_right'),
        AXIS.SHOULDER_AXIS_UPPER_RIGHT:  Axis(15, 'shoulder_axis_upper_right'),
        AXIS.SHOULDER_AXIS_MIDDLE_RIGHT: None
    }


class StickMapping:
    ps3_to_general = {
        # Stick mapping
        STICK.STICK_LEFT:  Stick(0, 1, 'stick_stick_left'),
        STICK.STICK_RIGHT: Stick(2, 3, 'stick_stick_right')
    }

            # self.axes.append(Axis(24, 'acc_x'))
            # self.axes.append(Axis(23, 'acc_y'))
            # self.axes.append(Axis(25, 'acc_z'))
            # self.axes.append(Axis(26, 'gyro_yaw'))



