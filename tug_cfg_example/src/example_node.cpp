#include <tug_cfg_example/example_node.h>
#include <ros/init.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/ros_param_reader.h>

namespace tug_cfg_example
{
ExampleNode::ExampleNode()
  : nh_("~"), config_server_(config_)
{
  config_server_.setLock(mutex_);
  config_server_.setCallback(std::bind(&ExampleNode::reconfigure, this));
}

ExampleNode::~ExampleNode()
{
}

void ExampleNode::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  tug_cfg::RosParamReader reader(nh_);
  tug_cfg::load(config_, reader);
  config_server_.start(nh_);
  timer_ = nh_.createTimer(ros::Duration(5.0), &ExampleNode::update, this);
  ROS_INFO("Running!");
}

void ExampleNode::update(const ros::TimerEvent&)
{
  std::unique_lock<std::mutex> lock(mutex_);
  ++config_.num_engineers;
  config_server_.notify();
}

void ExampleNode::reconfigure()
{
  ROS_INFO("Reconfigure!");
  if (config_.insane_mode)
  {
    ROS_ERROR("I'm quite sure you don't know what you're doing.");
    config_.insane_mode = false;
  }
}
}  // namespace tug_cfg_example



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tug_cfg_example");
  tug_cfg_example::ExampleNode node;
  node.start();
  ros::spin();
  return 0;
}
