#!/usr/bin/env python

from tug_joy_constants import *
from rospy import loginfo, logwarn, logerr, logdebug


class Actuator:
    def __init__(self, name='unknown', callback_fct=None, callback_filtering=CB_FILTERING_NONE):
        self.name = name
        self.callback_fct = callback_fct
        self.callback_filtering = callback_filtering
        self.change_since_last = 'None'

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = callback_filtering

    def __str__(self):
        return str(self.name)


class Button(Actuator):
    def __init__(self, button_id, name='unknown', callback_fct=None, callback_filtering=CB_FILTERING_NONE):
        Actuator.__init__(self, name, callback_fct, callback_filtering)
        self.value = 0.0
        self.button_id = button_id

    def set_value(self, msg):
        try:
            if self.value != msg.buttons[self.button_id]:
                self.value = msg.buttons[self.button_id]
                self.change_since_last = (CB_FILTERING_PRESS if self.value == 1 else CB_FILTERING_RELEASE)
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '", str(self.button_id), "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = callback_filtering

    def __str__(self):
        return ' B ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value) + ' filter: ' + self.callback_filtering

    def __repr__(self):
        return str(self)


class VirtualButton(Actuator):
    def __init__(self, axis_id, response_at_positive, name='unknown', callback_fct=None, callback_filtering=CB_FILTERING_NONE):
        Actuator.__init__(self, name, callback_fct, callback_filtering)
        self.value = 0.0
        self.axis_id = axis_id
        self.response_at_positive = response_at_positive

    def set_value(self, msg):
        try:
            new_value = round(msg.axes[self.axis_id])
            if self.response_at_positive:
                new_value = max(new_value, 0)
            else:
                new_value = max(new_value * -1, 0)

            if self.value != new_value:
                self.value = new_value
                self.change_since_last = (CB_FILTERING_PRESS if self.value == 1 else CB_FILTERING_RELEASE)
        except:
            logerr("[" + Actuator.__str__(self) + "] virtual button id '", str(self.axis_id), "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = callback_filtering

    def __str__(self):
        return 'VB ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value) + ' filter: ' + self.callback_filtering

    def __repr__(self):
        return str(self)


class Axis(Actuator):
    def __init__(self, axis_id, name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.value = 0.0
        self.axis_id = axis_id

    def set_value(self, msg):
        try:
            self.value = msg.axes[self.axis_id]
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '", str(self.axis_id), "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = CB_FILTERING_NONE
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at axes not possible!')

    def __str__(self):
        return ' A ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)


class VirtualAxis(Actuator):
    def __init__(self, positive_id, negative_id, use_axis, name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.value = 0.0
        self.positive_id = positive_id
        self.negative_id = negative_id

        if self.positive_id is not None and self.negative_id is None:
            self.set_value = self.set_value_with_axes_pos if use_axis else self.set_value_with_buttons_pos
        elif self.positive_id is None and self.negative_id is not None:
            self.set_value = self.set_value_with_axes_neg if use_axis else self.set_value_with_buttons_neg
        elif self.positive_id is not None and self.negative_id is not None:
            self.set_value = self.set_value_with_axes_pos_neg if use_axis else self.set_value_with_buttons_pos_neg

    def set_value_with_axes_pos(self, msg):
        try:
            self.value = abs(max(msg.axes[self.positive_id], 0.0))
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '" + str(self.positive_id) + "' not found")

    def set_value_with_axes_neg(self, msg):
        try:
            self.value = abs(min(msg.axes[self.negative_id], 0.0))
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '" + str(self.negative_id) + "' not found")

    def set_value_with_axes_pos_neg(self, msg):
        try:
            self.value = abs(msg.axes[self.positive_id]) - abs(msg.axes[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] one or more axis ids not found")

    def set_value_with_buttons_pos(self, msg):
        try:
            self.value = abs(msg.buttons[self.positive_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '" + str(self.positive_id) + "' not found")

    def set_value_with_buttons_neg(self, msg):
        try:
            self.value = abs(msg.buttons[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '" + str(self.negative_id) + "' not found")

    def set_value_with_buttons_pos_neg(self, msg):
        try:
            self.value = abs(msg.buttons[self.positive_id]) - abs(msg.buttons[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] one or more button ids not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = CB_FILTERING_NONE
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at axes not possible!')

    def __str__(self):
        return 'VA ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)


class Stick(Actuator):
    def __init__(self, horizontal_id, vertical_id, name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.horizontal_val = 0.0
        self.horizontal_id = horizontal_id
        self.vertical_val = 0.0
        self.vertical_id = vertical_id

    def set_value(self, msg):
        try:
            self.horizontal_val = msg.axes[self.horizontal_id]
            self.vertical_val = msg.axes[self.vertical_id]
        except:
            logerr("[" + Actuator.__str__(self) + "] stick id '", self.horizontal_id, "' or '", self.vertical_id, "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = CB_FILTERING_NONE
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at sticks not possible!')

    def __str__(self):
        return ' S ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)


class VirtualStickOf2(Actuator):
    def __init__(self, horizontal_actuator, vertical_actuator, invert_axes=[1.0, 1.0], name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.invert_axes = invert_axes
        self.horizontal_val = 0.0
        self.horizontal_actuator = horizontal_actuator
        self.vertical_val = 0.0
        self.vertical_actuator = vertical_actuator

    def set_value(self, msg):
        if not self.horizontal_actuator:
            logerr("[" + Actuator.__str__(self) + "] actuator '" + self.horizontal_actuator.name + "' not found")
            return
        if not self.vertical_actuator:
            logerr("[" + Actuator.__str__(self) + "] actuator '" + self.vertical_actuator.name + "' not found")
            return

        try:
            self.horizontal_val = self.horizontal_actuator.value
            self.horizontal_val *= self.invert_axes[0]
            self.vertical_val = self.vertical_actuator.value
            self.vertical_val *= self.invert_axes[1]
        except:
            logerr("[" + Actuator.__str__(self) + "] stick name '", self.horizontal_actuator.name, "' or '", self.vertical_actuator.name, "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = CB_FILTERING_NONE
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at virtual sticks not possible!')

    def __str__(self):
        return 'VS ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)


class VirtualStickOf4(Actuator):

    def __init__(self, up_actuator, right_actuator, down_actuator, left_actuator, invert_axes=[1.0, 1.0], name='unknown', callback_fct=None):
        Actuator.__init__(self, name, callback_fct)
        self.invert_axes = invert_axes
        self.horizontal_val = 0.0
        self.vertical_val = 0.0
        self.up_actuator = up_actuator
        self.right_actuator = right_actuator
        self.down_actuator = down_actuator
        self.left_actuator = left_actuator

    def set_value(self, msg):
        if not self.up_actuator:
            logerr("[" + Actuator.__str__(self) + "] actuator '" + self.up_actuator.name + "' not found")
            return
        if not self.right_actuator:
            logerr("[" + Actuator.__str__(self) + "] actuator '" + self.right_actuator.name + "' not found")
            return
        if not self.down_actuator:
            logerr("[" + Actuator.__str__(self) + "] actuator '" + self.down_actuator.name + "' not found")
            return
        if not self.left_actuator:
            logerr("[" + Actuator.__str__(self) + "] actuator '" + self.left_actuator.name + "' not found")
            return

        try:
            self.horizontal_val = self.left_actuator.value - self.right_actuator.value
            self.horizontal_val *= self.invert_axes[0]
            self.vertical_val = self.up_actuator.value - self.down_actuator.value
            self.vertical_val *= self.invert_axes[1]
        except:
            logerr("[" + Actuator.__str__(self) + "] button/axis not available or do not exist")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        self.callback_fct = callback_fct
        self.callback_filtering = CB_FILTERING_NONE
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at virtual sticks not possible!')

    def __str__(self):
        return 'VS ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)