#!/usr/bin/env python

import rospy
from joy_base import Manager


def limit(src, min_, max_):
    return max(min_, min(src, max_))


class AngularCommand:
    from std_msgs.msg import Float64

    def __init__(self, actuator, namespace, publishing_topic, inverse=False, absolute_mode=True):
        self.namespace_ = namespace
        self.init_angle_ = rospy.get_param('~' + self.namespace_ + 'init_angel', 0.0)
        self.min_ = rospy.get_param('~' + self.namespace_ + 'minimum_angle', -2.0)
        self.max_ = rospy.get_param('~' + self.namespace_ + 'maximum_angle', 2.0)
        self.std_delta_ = rospy.get_param('~' + self.namespace_ + 'std_delta', 0.02)
        self.max_speed_ = rospy.get_param('~' + self.namespace_ + 'max_speed', 1.0)
        self.actuator = actuator
        self.inverse = -1.0 if inverse else 1.0

        self.pup_ = rospy.Publisher(publishing_topic, self.Float64, queue_size=1)

        self.init_angle_ = limit(self.init_angle_, self.min_, self.max_)
        self.current_angle_ = self.init_angle_
        self.absolute_mode_ = absolute_mode

        self.callback = self.callback_absolute if self.absolute_mode_ else self.callback_relative

    def set_to_init(self, value_dict):
        self.current_angle_ = self.init_angle_
        self.pup_.publish(self.Float64(self.init_angle_))

    def callback_absolute(self, value_dict):
        # frequency = Manager().inst.frequency
        self.current_angle_ += self.std_delta_ * value_dict[self.actuator] * self.inverse
        self.current_angle_ = limit(self.current_angle_, self.min_, self.max_)
        self.pup_.publish(self.Float64(self.current_angle_))

    def callback_relative(self, value_dict):
        # frequency = Manager().inst.frequency
        self.current_angle_ = value_dict[self.actuator] * self.max_speed_ * self.inverse
        self.pup_.publish(self.Float64(self.current_angle_))

    def stop_cb(self, value_dict):
        self.pup_.publish(self.Float64(0.0))


class CmdVel:
    from geometry_msgs.msg import Twist

    def __init__(self, actuator_linear_x, actuator_linear_y, actuator_angular_z, namespace, publishing_topic,
                 trans_vel=0.5, rot_vel=0.5):

        self.namespace = namespace
        self.max_tv_ = rospy.get_param('~' + self.namespace + 'MaxTransVel', 1.0)
        self.min_tv_ = rospy.get_param('~' + self.namespace + 'MinTransVel', 0.1)
        self.max_rv_ = rospy.get_param('~' + self.namespace + 'MaxRotVel', 1.0)
        self.min_rv_ = rospy.get_param('~' + self.namespace + 'MinRotVel', 0.1)
        self.basic_tv_ = rospy.get_param('~' + self.namespace + 'InitTransVel', trans_vel)
        self.basic_rv_ = rospy.get_param('~' + self.namespace + 'InitRotVel', rot_vel)

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


class ServiceCall:
    def __init__(self, service_name, service_type, wait_for_service_timeout=10.0):
        self.service_type = service_type
        self.service_name = service_name
        self.service = None
        try:
            if wait_for_service_timeout:
                rospy.wait_for_service(self.service_name, wait_for_service_timeout)

            self.service = rospy.ServiceProxy(self.service_name, self.service_type)
        except Exception as e:
            rospy.logerr(e)

    def stop_cb(self, value_dict):
        pass

    def callback(self, service_value):
        rospy.logerr("ServiceCall::callback: " + str(service_value))
        try:
            resp = self.service(service_value)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


class TopicPublisher:
    def __init__(self, topic_name, topic_type, wait_for_service_timeout=10.0):
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.topic = rospy.Publisher(self.topic_name, self.topic_type, queue_size=10)

    def stop_cb(self, value_dict):
        pass

    def callback(self, topic_value):
        rospy.logerr("TopicPublisher::callback: " + str(topic_value))
        try:
            self.topic.publish(topic_value)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
