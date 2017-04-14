#include <tug_cfg_example/example_node.h>
#include <ros/init.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/ros_param_reader.h>

namespace tug_cfg_example
{
ExampleNode::ExampleNode()
  : nh_("~"), config_server_(config_)
{
  config_server_.setCallback(std::bind(&ExampleNode::reconfigure, this));
}

ExampleNode::~ExampleNode()
{
}

void ExampleNode::run()
{
  tug_cfg::RosParamReader reader(nh_);
  tug_cfg::load(config_, reader);
  config_server_.start(nh_);
  ROS_INFO("Run!");
  ros::spin();
}

void ExampleNode::reconfigure()
{
  ROS_INFO("Reconfigure!");
}
}  // namespace tug_cfg_example



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tug_cfg_example");
  tug_cfg_example::ExampleNode node;
  node.run();
  return 0;
}
