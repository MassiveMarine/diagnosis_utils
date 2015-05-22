#!/usr/bin/env python

from tug_joy_constants import *
from rospy import loginfo, logwarn, logerr, logdebug


class Actuator:
    """
    Base of all actuators.
    """
    def __init__(self, name='unknown', callback_fct=None, callback_filtering=CB_FILTERING_NONE):
        """
        Constructor of Actuator
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: only necessary if actuator is a button.
        """
        self.name = name
        self.callback_fct = callback_fct
        self.callback_filtering = callback_filtering
        self.change_since_last = 'None'

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        """
        Is called to setup the callback function and the callback filter.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: only necessary if actuator is a button.
        """
        self.callback_fct = callback_fct
        self.callback_filtering = callback_filtering

    def __str__(self):
        return str(self.name)


class Button(Actuator):
    """
    Derivation from actuator. This class is for buttons, which only can be
    1 (pressed) or 0 (not pressed). It extracts its data from the 'buttons'-
    array of the joy-msg.
    """
    def __init__(self, button_id, name='unknown', callback_fct=None, callback_filtering=CB_FILTERING_NONE):
        """
        Constructor of Button derived from Actuator
        :param button_id: index of this button in the 'buttons'-array of the joy-msg.
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering:
                CB_FILTERING_NONE    -> callback is always called
                CB_FILTERING_PRESS   -> callback is only called once if button has been pressed
                CB_FILTERING_RELEASE -> callback is only called once if button has been released
        """
        Actuator.__init__(self, name, callback_fct, callback_filtering)
        self.value = 0.0
        self.button_id = button_id

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the value of the
        actuator. It reads the value of the 'Buttons'-array of the joy-msg at
        its corresponding index. Additionally information for the filter are defined.
        :param msg: ROS subscriber message
        """
        try:
            if self.value != msg.buttons[self.button_id]:
                self.value = msg.buttons[self.button_id]
                self.change_since_last = (CB_FILTERING_PRESS if self.value == 1 else CB_FILTERING_RELEASE)
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '", str(self.button_id), "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        """
        Is called to setup the callback function and the callback filter.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering:
                CB_FILTERING_NONE    -> callback is always called
                CB_FILTERING_PRESS   -> callback is only called once if button has been pressed
                CB_FILTERING_RELEASE -> callback is only called once if button has been released
        """
        Actuator.set_cb(self, callback_fct, callback_filtering)

    def __str__(self):
        return ' B ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value) + ' filter: ' + self.callback_filtering

    def __repr__(self):
        return str(self)


class VirtualButton(Actuator):
    """
    Derivation from actuator. This class is for buttons, which only can be
    1 (pressed) or 0 (not pressed). It extracts its data from the 'axis'-
    array of the joy-msg and rounds the float values to simulate buttons.
    """
    def __init__(self, axis_id, response_at_positive, name='unknown', callback_fct=None, callback_filtering=CB_FILTERING_NONE):
        """
        Constructor of Button derived from Actuator
        :param axis_id: index of this button in the 'axes'-array of the joy-msg.
        :param response_at_positive: True  if >  0.5 represents a pressed button
                                     False if < -0.5 represents a pressed button
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering:
                CB_FILTERING_NONE    -> callback is always called
                CB_FILTERING_PRESS   -> callback is only called once if button has been pressed
                CB_FILTERING_RELEASE -> callback is only called once if button has been released

        """
        Actuator.__init__(self, name, callback_fct, callback_filtering)
        self.value = 0.0
        self.axis_id = axis_id
        self.response_at_positive = response_at_positive

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the value of the
        actuator. It reads the value of the 'Axes'-array of the joy-msg at
        its corresponding index, rounds the floating value and checks if the
        virtual button is for positive or negative values. Additionally
        information for the filter are defined.
        :param msg: ROS subscriber message
        """
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
        """
        Is called to setup the callback function and the callback filter.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering:
                CB_FILTERING_NONE    -> callback is always called
                CB_FILTERING_PRESS   -> callback is only called once if button has been pressed
                CB_FILTERING_RELEASE -> callback is only called once if button has been released
        """
        Actuator.set_cb(self, callback_fct, callback_filtering)

    def __str__(self):
        return 'VB ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value) + ' filter: ' + self.callback_filtering

    def __repr__(self):
        return str(self)


class Axis(Actuator):
    """
    Derivation from actuator. This class is for axis, which can be floating
    values between -1.0 and 1.0. It extracts its data from the 'axis'-array
    of the joy-msg.
    """
    def __init__(self, axis_id, name='unknown', callback_fct=None):
        """
        Constructor of Axis derived from Actuator
        :param axis_id: index of this button in the 'axes'-array of the joy-msg.
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        """
        Actuator.__init__(self, name, callback_fct)
        self.value = 0.0
        self.axis_id = axis_id

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the value of the
        actuator. It reads the value of the 'Axes'-array of the joy-msg at
        its corresponding index.
        :param msg: ROS subscriber message
        """
        try:
            self.value = msg.axes[self.axis_id]
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '", str(self.axis_id), "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        """
        Is called to setup the callback function.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: This is ignored and always set to 'CB_FILTERING_NONE'.
        """
        Actuator.set_cb(self, callback_fct, CB_FILTERING_NONE)
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at axes not possible!')

    def __str__(self):
        return ' A ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)


class VirtualAxis(Actuator):
    """
    Derivation from actuator. This class is for axis, which can be floating
    values between -1.0 and 1.0. It extracts its data from two inputs, which
    can be buttons or axes.
    """
    def __init__(self, positive_id, negative_id, use_axis, name='unknown', callback_fct=None):
        """
        Constructor of Axis derived from Actuator
        :param positive_id: Index of the actuator in the 'buttons'-array or 'axes'-array of the joy-msg.
                            This actuator is used to generate the positive values of the virtual axis.
                            If id is None only the negative id is used.
        :param negative_id: Index of the actuator in the 'buttons'-array or 'axes'-array of the joy-msg.
                            This actuator is used to generate the negative values of the virtual axis.
                            If id is None only the positive id is used.
        :param use_axis: Defines if the ids are for the 'buttons'-array or the 'axes'-array of the joy-msg.
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        """
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
        """
        Set value if only positive id is set and axes are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(max(msg.axes[self.positive_id], 0.0))
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '" + str(self.positive_id) + "' not found")

    def set_value_with_axes_neg(self, msg):
        """
        Set value if only negative id is set and axes are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(min(msg.axes[self.negative_id], 0.0))
        except:
            logerr("[" + Actuator.__str__(self) + "] axis id '" + str(self.negative_id) + "' not found")

    def set_value_with_axes_pos_neg(self, msg):
        """
        Set value if positive and negative id are set and axes are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.axes[self.positive_id]) - abs(msg.axes[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] one or more axis ids not found")

    def set_value_with_buttons_pos(self, msg):
        """
        Set value if only positive id is set and buttons are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.buttons[self.positive_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '" + str(self.positive_id) + "' not found")

    def set_value_with_buttons_neg(self, msg):
        """
        Set value if only negative id is set and buttons are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.buttons[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] button id '" + str(self.negative_id) + "' not found")

    def set_value_with_buttons_pos_neg(self, msg):
        """
        Set value if positive and negative ids are set and buttons are used as input.
        :param msg: ROS subscriber message
        """
        try:
            self.value = abs(msg.buttons[self.positive_id]) - abs(msg.buttons[self.negative_id])
        except:
            logerr("[" + Actuator.__str__(self) + "] one or more button ids not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        """
        Is called to setup the callback function.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: This is ignored and always set to 'CB_FILTERING_NONE'.
        """
        Actuator.set_cb(self, callback_fct, CB_FILTERING_NONE)
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at axes not possible!')

    def __str__(self):
        return 'VA ' + Actuator.__str__(self) + ' val: {0: .3f}'.format(self.value)

    def __repr__(self):
        return str(self)


class Stick(Actuator):
    """
    Derivation from actuator. This class is for sticks, which can be floating
    values between -1.0 and 1.0 for the horizontal and also for the vertical
    axes. It extracts its data from two axes.
    """
    def __init__(self, horizontal_id, vertical_id, name='unknown', callback_fct=None):
        """
        Constructor of Sticks derived from Actuator
        :param horizontal_id: Index of the actuator in the 'axes'-array of the joy-msg.
                              This actuator is used to generate the horizontal values of the stick.
        :param vertical_id:   Index of the actuator in the 'axes'-array of the joy-msg.
                              This actuator is used to generate the vertical values of the stick.
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        """
        Actuator.__init__(self, name, callback_fct)
        self.horizontal_val = 0.0
        self.horizontal_id = horizontal_id
        self.vertical_val = 0.0
        self.vertical_id = vertical_id

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the values of the
        actuator. It reads the values of the 'Axes'-array of the joy-msg at
        its corresponding indices.
        :param msg: ROS subscriber message
        """
        try:
            self.horizontal_val = msg.axes[self.horizontal_id]
            self.vertical_val = msg.axes[self.vertical_id]
        except:
            logerr("[" + Actuator.__str__(self) + "] stick id '", self.horizontal_id, "' or '", self.vertical_id, "' not found")

    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        """
        Is called to setup the callback function.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: This is ignored and always set to 'CB_FILTERING_NONE'.
        """
        Actuator.set_cb(self, callback_fct, CB_FILTERING_NONE)
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at sticks not possible!')

    def __str__(self):
        return ' S ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)


class VirtualStickOf2(Actuator):
    """
    Derivation from actuator. This class is for sticks, which can be floating
    or integer values between -1.0 and 1.0 for the horizontal and also for the
    vertical axes. It extracts its data from two given actuators. It does not
    use the joy-msg.
    """
    def __init__(self, horizontal_actuator, vertical_actuator, invert_axes=[1.0, 1.0], name='unknown', callback_fct=None):
        """
        Constructor of Sticks derived from Actuator
        :param horizontal_actuator: Actuator to generate the horizontal values of the stick.
        :param vertical_actuator: Actuator to generate the vertical values of the stick.
        :param invert_axes: Can be set to invert (and scale) the horizontal and
               vertical behaviour of the stick. (1.0 => no inverting, -1.0 => inverting)
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        """
        Actuator.__init__(self, name, callback_fct)
        self.invert_axes = invert_axes
        self.horizontal_val = 0.0
        self.horizontal_actuator = horizontal_actuator
        self.vertical_val = 0.0
        self.vertical_actuator = vertical_actuator

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the values of the
        actuator. It reads the values of the two actuators and generates a
        virtual stick.
        :param msg: ROS subscriber message BUT NOT USED
        """
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
        """
        Is called to setup the callback function.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: This is ignored and always set to 'CB_FILTERING_NONE'.
        """
        Actuator.set_cb(self, callback_fct, CB_FILTERING_NONE)
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at virtual sticks not possible!')

    def __str__(self):
        return 'VS ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)


class VirtualStickOf4(Actuator):
    """
    Derivation from actuator. This class is for sticks, which can be floating
    or integer values between -1.0 and 1.0 for the horizontal and also for the
    vertical axes. It extracts its data from four given actuators. It does not
    use the joy-msg.
    """
    def __init__(self, up_actuator, right_actuator, down_actuator, left_actuator, invert_axes=[1.0, 1.0], name='unknown', callback_fct=None):
        """
        Constructor of Sticks derived from Actuator
        :param up_actuator: Actuator to generate the up-values of the stick.
        :param right_actuator: Actuator to generate the right-values of the stick.
        :param down_actuator: Actuator to generate the down-values of the stick.
        :param left_actuator: Actuator to generate the left-values of the stick.
        :param invert_axes: Can be set to invert (and scale) the horizontal and
               vertical behaviour of the stick. (1.0 => no inverting, -1.0 => inverting)
        :param name: name of this actuator
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        """
        Actuator.__init__(self, name, callback_fct)
        self.invert_axes = invert_axes
        self.horizontal_val = 0.0
        self.vertical_val = 0.0
        self.up_actuator = up_actuator
        self.right_actuator = right_actuator
        self.down_actuator = down_actuator
        self.left_actuator = left_actuator

    def set_value(self, msg):
        """
        Is called from the subscriber callback and updates the values of the
        actuator. It reads the values of the four actuators and generates a
        virtual stick.
        :param msg: ROS subscriber message BUT NOT USED
        """
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
        """
        Is called to setup the callback function.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: This is ignored and always set to 'CB_FILTERING_NONE'.
        """
        Actuator.set_cb(self, callback_fct, CB_FILTERING_NONE)
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at virtual sticks not possible!')

    def __str__(self):
        return 'VS ' + Actuator.__str__(self) + ' h: {0: .3f}'.format(self.horizontal_val) + ' v: {0: .3f}'.format(self.vertical_val)

    def __repr__(self):
        return str(self)













class Group(Actuator):
    def __init__(self, actuators, name='unknown', callback_fct=None):

        Actuator.__init__(self, name, callback_fct)
        self.actuators = actuators
        self.__values = dict((actuator.name, 0.0) for actuator in actuators)

    def set_value(self, msg):

        if not all(self.actuators):
            logerr("[" + Actuator.__str__(self) + "] actuator not found")
            return

        try:
            for actuator in self.actuators:
                self.__values[actuator.name] = actuator.value

        except:
            logerr("[" + Actuator.__str__(self) + "] button/axis not available or do not exist")

    def get_value(self, actuator_name):
        if actuator_name not in self.__values:
            raise ValueError("[" + str(self.name) + "] Actuator '" + str(actuator_name) + "' not used in this group!")

        return self.__values[actuator_name]


    def set_cb(self, callback_fct, callback_filtering=CB_FILTERING_NONE):
        """
        Is called to setup the callback function.
        :param callback_fct: callback function, which can be set by the user and is called by the manager.
        :param callback_filtering: This is ignored and always set to 'CB_FILTERING_NONE'.
        """
        Actuator.set_cb(self, callback_fct, CB_FILTERING_NONE)
        if callback_filtering != CB_FILTERING_NONE:
            logwarn('CB Filtering at actuator groups not possible!')

    def __str__(self):
        return ' G ' + Actuator.__str__(self) + '\n' + '\n'.join(str(key) + ': ' + str(val) for key, val in self.__values.iteritems())

    def __repr__(self):
        return str(self)