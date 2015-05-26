#!/usr/bin/env python

import rospy


class CmdVel:
    from geometry_msgs.msg import Twist

    def __init__(self, actuator_linear_x, actuator_linear_y, actuator_angular_z, namespace, publishing_topic):

        self.namespace = namespace
        self.max_tv_ = rospy.get_param('~' + self.namespace + 'MaxTransVel', 1.0)
        self.min_tv_ = rospy.get_param('~' + self.namespace + 'MinTransVel', 1.0)
        self.max_rv_ = rospy.get_param('~' + self.namespace + 'MaxRotVel', 1.0)
        self.min_rv_ = rospy.get_param('~' + self.namespace + 'MinRotVel', 1.0)
        self.basic_tv_ = rospy.get_param('~' + self.namespace + 'InitTransVel', 1.0)
        self.basic_rv_ = rospy.get_param('~' + self.namespace + 'InitRotVel', 1.0)

        self.actuator_linear_x = actuator_linear_x
        self.actuator_linear_y = actuator_linear_y
        self.actuator_angular_z = actuator_angular_z

        self.used_actuators = []
        if actuator_linear_x:
            self.used_actuators.append(actuator_linear_x)
        if actuator_linear_y:
            self.used_actuators.append(actuator_linear_y)
        if actuator_angular_z:
            self.used_actuators.append(actuator_angular_z)

        self.cmd_vel_pub_ = rospy.Publisher(publishing_topic, self.Twist, queue_size=1)

    def increase_linear_speed_cb(self, value_dict):
        self.basic_tv_ += self.max_tv_ / 10
        if self.basic_tv_ > self.max_tv_:
            self.basic_tv_ = self.max_tv_

    def decrease_linear_speed_cb(self, value_dict):
        self.basic_tv_ -= self.max_tv_ / 10
        if self.basic_tv_ < self.min_tv_:
            self.basic_tv_ = self.min_tv_

    def increase_angular_speed_cb(self, value_dict):
        self.basic_rv_ += self.max_rv_ / 10
        if self.basic_rv_ > self.max_rv_:
            self.basic_rv_ = self.max_rv_

    def decrease_angular_speed_cb(self, value_dict):
        self.basic_rv_ -= self.max_rv_ / 10
        if self.basic_rv_ < self.min_rv_:
            self.basic_rv_ = self.min_rv_

    def callback(self, value_dict):
        new_twist = self.Twist()
        try:
            print value_dict
            if self.actuator_linear_x:
                new_twist.linear.x = self.basic_tv_ * value_dict[self.actuator_linear_x]
            if self.actuator_linear_y:
                new_twist.linear.y = self.basic_tv_ * value_dict[self.actuator_linear_y]
            if self.actuator_angular_z:
                new_twist.angular.z = self.basic_rv_ * value_dict[self.actuator_angular_z]

            self.cmd_vel_pub_.publish(new_twist)

        except ValueError as error:
            rospy.logerr(error)


