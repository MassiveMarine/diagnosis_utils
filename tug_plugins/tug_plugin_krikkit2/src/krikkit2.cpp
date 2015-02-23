#include <tug_plugin_krikkit2/krikkit2.h>

#include <boost/make_shared.hpp>
#include <nav_msgs/Odometry.h>

#define DEG2RAD (M_PI / 180.0)
#define CONV_ODO_LIN    (1.0/(1000.0))  // get mm/s, use m/s
#define CONV_ODO_ROT    (DEG2RAD/1000.0)  // get m°/s, use rad/s

const uint32_t CONV_MOTION_LIN = 1000.0;              // get m/s, send mm/s
const uint32_t CONV_MOTION_ROT = 1000.0;              // get rad/s, send (mrad/s)

const double TWISTCOV = 0.1; // covariance of the dr, ds and dphi odometry measurements

const uint32_t MESSAGEQUEUESIZE = 5; // only keep newest information
const uint32_t CAN_KRIKKIT_DEVICE_ID = 2;

//! Message specifying the motion data the drive should follow (e.g. dr/ds/dphi)
//! Data-Format: 6 Byte, dr [mm/sec], ds [mm/sec], dphi*1000 [rad/sec]
const uint32_t CAN_MSG_VELOCITY_COMMAND = 0x07;

//! Message which will be sent from the hardware with odometry data
//! Data-Format: 6 Bytes, dr [mm/sec], ds [mm/sec], dphi*1000 [rad/sec]
const uint32_t CAN_MSG_ODOMETRY_DATA = 0x06;

//! Message number for enabling the odometry
//! Data-Format: 1 Byte, 0 ... OFF, 1 ... ON
const uint32_t CAN_MSG_ENABLE_ODOMETRY = 0x05;

namespace tug_plugin_krikkit2
{
  Krikkit2::Krikkit2() :
      node_handle_("~"), odometry_seq_count_(1), current_time_(ros::Time::now()), last_time_(ros::Time::now()), x_(0.0), y_(0.0), th_(0.0)
  {
  }

  void Krikkit2::initialize(tug_robot_control::RobotHardware* robot_hardware, const ros::NodeHandle & nh, std::string name)
  {
    ROS_INFO("Krikkit2::initialize");
    cmd_vel_subscriber_ = node_handle_.subscribe("/cmd_vel", 1, &Krikkit2::cmdVelCallback, this);
    odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("/odom", 1);

    can_interface_ = boost::make_shared<tug_can_interface::CanNicDriverHost>("tug_can_nic_drivers::SocketCanNicDriver", "can0", 0, ros::Duration(1.0),
        ros::NodeHandle(node_handle_, "can_nic_driver"));

    odom_subscriber_ = can_interface_->subscribeToAll(boost::bind(&Krikkit2::odometryCallback, this, _1));

    enableOdometrySensor();
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;

  }

  void Krikkit2::cmdVelCallback(const geometry_msgs::TwistConstPtr & cmd_vel)
  {
    //ROS_INFO_STREAM("Krikkit::cmdVelCallback");

    // create and init CAN message
    tug_can_msgs::CanMessagePtr can_message = boost::make_shared<tug_can_msgs::CanMessage>();
    can_message->time = ros::Time::now();
    can_message->id = getMessageHeader(CAN_KRIKKIT_DEVICE_ID, CAN_MSG_VELOCITY_COMMAND);
    can_message->rtr = false;
    can_message->extended = true;

    // calculate CAN message values
    uint16_t dr = static_cast<uint16_t>(cmd_vel->linear.x * CONV_MOTION_LIN);
    uint16_t ds = static_cast<uint16_t>(cmd_vel->linear.y * CONV_MOTION_LIN);
    uint16_t dp = static_cast<uint16_t>(cmd_vel->angular.z * CONV_MOTION_ROT);

    //ROS_INFO_STREAM("Received velocity command dr = " << dr << ", ds = " << ds << ", dphi = " << dp);

    // pack high level velocity commands into CAN message byte array
    uint8_t dr_raw[2];
    Conversion::uint16Touint8(dr, dr_raw);
    uint8_t ds_raw[2];
    Conversion::uint16Touint8(ds, ds_raw);
    uint8_t dp_raw[2];
    Conversion::uint16Touint8(dp, dp_raw);

    // add data to CAN message byte array
    can_message->data.push_back(dr_raw[0]);
    can_message->data.push_back(dr_raw[1]);
    can_message->data.push_back(ds_raw[0]);
    can_message->data.push_back(ds_raw[1]);
    can_message->data.push_back(dp_raw[0]);
    can_message->data.push_back(dp_raw[1]);

    //ROS_INFO_STREAM("Publish to can node:\n" << *can_message);

    can_interface_->sendMessage(can_message);
  }

  void Krikkit2::odometryCallback(const tug_can_msgs::CanMessageConstPtr & msg)
  {
    boost::uint8_t dr_data[2];
    boost::uint8_t ds_data[2];
    boost::uint8_t dp_data[2];

    // get data from the can message
    dr_data[0] = msg->data[0];
    dr_data[1] = msg->data[1];
    ds_data[0] = msg->data[2];
    ds_data[1] = msg->data[3];
    dp_data[0] = msg->data[4];
    dp_data[1] = msg->data[5];

    // unpack the data
    uint16_t dr;
    uint16_t ds;
    uint16_t dp;
    Conversion::uint8Touint16(dr_data, dr);
    Conversion::uint8Touint16(ds_data, ds);
    Conversion::uint8Touint16(dp_data, dp);

    // convert the data from the can message to double values of the odometry
    double vx;
    double vy;
    double vth;
    vx = static_cast<double>(dr) * static_cast<double>(CONV_ODO_LIN);
    vy = static_cast<double>(ds) * static_cast<double>(CONV_ODO_LIN);
    vth = static_cast<double>(dp) * static_cast<double>(CONV_ODO_ROT);

    // calculate odometry data
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();
    double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
    double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
    double delta_th = vth * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);

    // publish local odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = current_time_;
    odometry.header.seq = odometry_seq_count_;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";

    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.linear.y = vy;
    odometry.twist.twist.angular.z = vth;
    odometry.twist.covariance[0] = TWISTCOV;
    odometry.twist.covariance[7] = TWISTCOV;
    odometry.twist.covariance[14] = TWISTCOV;
    odometry.twist.covariance[21] = TWISTCOV;
    odometry.twist.covariance[28] = TWISTCOV;
    odometry.twist.covariance[35] = TWISTCOV;

    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = odom_quat;

    odom_publisher_.publish(odometry);

    // increase number of received odometry data
    ++odometry_seq_count_;
    last_time_ = current_time_;

  }

  void Krikkit2::enableOdometrySensor()
  {
    ROS_INFO_STREAM("Krikkit::enableOdometrySensor");
    tug_can_msgs::CanMessagePtr can_message = boost::make_shared<tug_can_msgs::CanMessage>();
    can_message->time = ros::Time::now();
    can_message->id = getMessageHeader(CAN_KRIKKIT_DEVICE_ID, CAN_MSG_ENABLE_ODOMETRY);
    can_message->data.push_back(1); // enable
    can_message->rtr = false;
    can_message->extended = true;
    can_interface_->sendMessage(can_message);
  }

  uint32_t Krikkit2::getMessageHeader(uint32_t device_id, uint32_t message_id)
  {
    return ((message_id << 20) | (0x1 << device_id));
  }

  void Conversion::uint8Touint16(uint8_t src_value[2], uint16_t & dest_value)
  {
    dest_value = ((src_value[1] << 8) | src_value[0]);
  }

  //--------------------------------------------------------------------------------
  void Conversion::uint16Touint8(uint16_t src_value, uint8_t (&dest_value)[2])
  {
    dest_value[0] = (0x00FF & src_value);
    dest_value[1] = (0xFF00 & src_value) >> 8;
  }
}

PLUGINLIB_EXPORT_CLASS(tug_plugin_krikkit2::Krikkit2, tug_plugin_manager::RegularPlugin)
