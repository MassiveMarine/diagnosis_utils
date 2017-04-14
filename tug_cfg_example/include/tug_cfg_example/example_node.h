#ifndef _TUG_CFG_EXAMPLE__EXAMPLE_NODE_H_
#define _TUG_CFG_EXAMPLE__EXAMPLE_NODE_H_

#include <ros/node_handle.h>
#include <tug_cfg/ros_dynamic_config_server.h>
#include <tug_cfg_example/ExampleConfig.h>

namespace tug_cfg_example
{
class ExampleNode
{
public:
  ExampleNode();
  virtual ~ExampleNode();

  void run();

protected:
  void reconfigure();

  ros::NodeHandle nh_;
  ExampleConfig config_;
  tug_cfg::RosDynamicConfigServer config_server_;
};
}  // namespace tug_cfg_example

#endif  // _TUG_CFG_EXAMPLE__EXAMPLE_NODE_H_
