#include <tug_cfg_example/example_node.h>
#include <ros/init.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/ros_param_reader.h>

namespace tug_cfg_example
{
ExampleNode::ExampleNode()
  : nh_("~")
{
}

ExampleNode::~ExampleNode()
{
}

void ExampleNode::run()
{
  tug_cfg::RosParamReader reader(nh_);
  tug_cfg::load(config_, reader);
  ROS_INFO("Yay!");
  ros::spin();
}
}  // namespace tug_cfg_example



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tug_cfg_example");
  tug_cfg_example::ExampleNode node;
  node.run();
  return 0;
}
