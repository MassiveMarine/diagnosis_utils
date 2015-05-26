#!/usr/bin/env python

"""
Main file which starts the joy manager, load the controller-to-general mapping,
and executes the 'start_cb' which has to be defined by the user in 'tug_joy_own_config'
"""

import rospy
from sensor_msgs.msg import Joy
from tug_joy_base import Manager
from tug_joy_constants import *

import tug_joy_own_config as config

if __name__ == "__main__":
    rospy.init_node('tug_joy', anonymous=True)

    joy_topic_name = rospy.get_param('~joy_topic_name', '/joy')
    joy_manager_frequency = rospy.get_param('~joy_manager_frequency', '/joy')
    controller_name = rospy.get_param('~joy_controller_name', CONTROLLER.CONTROLLER_NAME_DEFAULT)
    rospy.loginfo('loaded controller is ' + controller_name)

    try:
        Manager(controller_name, joy_manager_frequency,
                config.manager_break_once_cb,
                config.manager_break_continuous_cb,
                config.manager_exit_cb)

        # config.special_actuators()
        config.manager_start_cb()


        joy_sub = rospy.Subscriber(joy_topic_name, Joy, Manager().joy_callback)
        Manager().run(joy_sub)

    except KeyboardInterrupt:
        pass
    except ValueError as error:
        rospy.logerr(error)
    except rospy.ROSInterruptException:
        pass
