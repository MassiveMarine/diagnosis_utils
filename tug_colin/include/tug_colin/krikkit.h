#ifndef KRIKKIT_H_
#define KRIKKIT_H_

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <tug_can_interface/can_nic_driver_host.h>

namespace tug_krikkit
{
class Krikkit
{
public:
  Krikkit();
  int run();

private:
  ros::NodeHandle node_handle_;

  ros::Subscriber cmd_vel_subscriber_;
  void cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel);

  ros::Publisher odom_publisher_;

  void odometryCallback(const tug_can_msgs::CanMessageConstPtr & msg);
  tug_can_interface::CanSubscriptionPtr odom_subscriber_;

  void enableOdometrySensor();
  uint32_t getMessageHeader(uint32_t device_id, uint32_t message_id);

  tug_can_interface::CanNicDriverHostPtr can_interface_;

  unsigned int odometry_seq_count_;
};
}

#endif /* KRIKKIT_H_ */
