#ifndef TUG_PLUGIN_KRIKKIT2_H_
#define TUG_PLUGIN_KRIKKIT2_H_

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>

// Select necessary plugin base
//#include <tug_robot_control/tug_controlled_device_driver_base.h>
#include <tug_robot_control/tug_device_driver_base.h>
//#include <tug_robot_control/tug_preprocessor_base.h>
//#include <tug_robot_control/tug_postprocessor_base.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tug_can_interface/can_nic_driver_host.h> //only while can is not available as plugin

namespace tug_plugin_krikkit2
{

  class Krikkit2: public tug_robot_control::DeviceDriver
  {
  public:
    Krikkit2();

    virtual void initialize(tug_robot_control::RobotHardware* robot_hardware, const ros::NodeHandle & nh, std::string name);

  private:
    ros::NodeHandle node_handle_;

    ros::Subscriber cmd_vel_subscriber_;
    void cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel);

    ros::Publisher odom_publisher_;

    void odometryCallback(const tug_can_msgs::CanMessageConstPtr & msg);
    tug_can_interface::CanSubscriptionPtr odom_subscriber_;

    void enableOdometrySensor();
    uint32_t getMessageHeader(uint32_t device_id, uint32_t message_id);

    tug_can_interface::CanNicDriverHostPtr can_interface_; //only while can is not available as plugin

    unsigned int odometry_seq_count_;
    tf::TransformBroadcaster odom_broadcaster_;
    double x_, y_, th_;
    ros::Time current_time_, last_time_;

  };

  class Conversion
  {
  public:
    //!-------------------------------------------------------------------------------
    //! uint16Touint8
    //! converts a uint16 into two uint8 (Little-Endian)
    //! @param src_value: the source value of the conversion
    //! @param dest_value: the destination value of the conversion
    static void uint16Touint8(uint16_t src_value, uint8_t (&dest_value)[2]);

    //!-------------------------------------------------------------------------------
    //! uint8Touint16
    //! converts from two uint8 into uint16 (Little-Endian)
    //! @param src_value: the source value of the conversion
    //! @param dest_value: the destination value of the conversion
    static void uint8Touint16(uint8_t src_value[2], uint16_t& dest_value);
  };
}

#endif /* TUG_PLUGIN_KRIKKIT2_H_ */
