#ifndef _TUG_CFG__ROS_DYNAMIC_CONFIGURATION_SERVER_H_
#define _TUG_CFG__ROS_DYNAMIC_CONFIGURATION_SERVER_H_

#include <functional>
#include <ros/node_handle.h>
#include <tug_cfg/struct.h>

namespace tug_cfg
{
class RosDynamicConfigurationServer
{
public:
  RosDynamicConfigurationServer();
  RosDynamicConfigurationServer(tug_cfg::AbstractStruct& configuration, const ros::NodeHandle& node_handle);
  virtual ~RosDynamicConfigurationServer();

  void start(tug_cfg::AbstractStruct& configuration, const ros::NodeHandle& node_handle);
  void stop();

  void setCallback(const std::function<void()>& callback);

  /**
   * Publishes current state of configuration. Used to notify e.g. GUI of changes.
   */
  void notify();

protected:
  RosDynamicConfigurationServer(const RosDynamicConfigurationServer&) = delete;
  RosDynamicConfigurationServer& operator=(const RosDynamicConfigurationServer&) = delete;

  tug_cfg::AbstractStruct* configuration_;
  ros::ServiceServer srv_;
  ros::Publisher descriptions_pub_;
  ros::Publisher updates_pub_;
  std::function<void> callback_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__ROS_DYNAMIC_CONFIGURATION_SERVER_H_
