#!/usr/bin/env python

# basics
import rospy
from sensor_msgs.msg import Joy
# from tug_joy_base import Mapping
from tug_joy_base import Manager
from tug_joy_constants import *

import tug_joy_own_config

manager = None
if __name__ == "__main__":
    rospy.init_node('tug_joy', anonymous=True)

    # if rospy.has_param('joy_topic_name'):
    joy_topic_name = rospy.get_param('~joy_topic_name', '/joy')
    joy_manager_frequency = rospy.get_param('~joy_manager_frequency', '/joy')
    controller_name = rospy.get_param('~joy_controller_name', CONTROLLER.CONTROLLER_NAME_DEFAULT)

    try:
        Manager(controller_name, joy_manager_frequency,
                tug_joy_own_config.manager_break_once_cb,
                tug_joy_own_config.manager_break_continuous_cb,
                tug_joy_own_config.manager_exit_cb)

        Manager().set_function_mapping(tug_joy_own_config.startup_config())
        tug_joy_own_config.special_actuators()

        joy_sub = rospy.Subscriber(joy_topic_name, Joy, Manager().joy_callback)
        Manager().run(joy_sub)

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
