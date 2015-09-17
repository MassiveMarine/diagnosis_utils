#!/usr/bin/env python

import rospy


def limit(src, min_, max_):
    return max(min_, min(src, max_))


class AngularCommand:
    from std_msgs.msg import Float64

    def __init__(self, actuator, namespace, publishing_topic, inverse=False):
        self.namespace_ = namespace
        self.init_angle_ = rospy.get_param('~' + self.namespace_ + 'init_angel', 0.0)
        self.min_ = rospy.get_param('~' + self.namespace_ + 'minimum_angle', -2.0)
        self.max_ = rospy.get_param('~' + self.namespace_ + 'maximum_angle',  2.0)
        self.std_delta_ = rospy.get_param('~' + self.namespace_ + 'std_delta',  0.02)
        self.actuator_ = actuator
        self.inverse = -1.0 if inverse else 1.0

        self.pup_ = rospy.Publisher(publishing_topic, self.Float64, queue_size=1)

        self.init_angle_ = limit(self.init_angle_, self.min_, self.max_)
        self.current_angle_ = self.init_angle_

    def set_to_init(self, value_dict):
        self.current_angle_ = self.init_angle_
        self.pup_.publish(self.Float64(self.init_angle_))

    def callback(self, value_dict):
        self.current_angle_ += self.std_delta_ * value_dict[self.actuator_] * self.inverse
        self.current_angle_ = limit(self.current_angle_, self.min_, self.max_)
        self.pup_.publish(self.Float64(self.current_angle_))


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

    def stop_cb(self, value_dict):
        new_twist = self.Twist()
        self.cmd_vel_pub_.publish(new_twist)

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


