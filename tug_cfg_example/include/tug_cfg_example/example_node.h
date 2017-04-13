#ifndef _TUG_CFG_EXAMPLE__EXAMPLE_NODE_H_
#define _TUG_CFG_EXAMPLE__EXAMPLE_NODE_H_

#include <ros/node_handle.h>
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
  ros::NodeHandle nh_;
  ExampleConfig config_;
};
}  // namespace tug_cfg_example

#endif  // _TUG_CFG_EXAMPLE__EXAMPLE_NODE_H_
