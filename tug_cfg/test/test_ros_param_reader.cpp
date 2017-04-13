#include <gtest/gtest.h>
#include <limits>
#include <ros/init.h>
#include <tug_cfg/BarConfig.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/ros_param_reader.h>
#include "test_helpers.h"

TEST(TestBarConfig, testLoadRosParam1)
{
  tug_cfg::BarConfig config;
  tug_cfg::RosParamReader reader(ros::NodeHandle("~"), "bar1");
  tug_cfg::load(config, reader);
  assertBar1LoadedCorrectly(config);
}

TEST(TestBarConfig, testLoadRosParam2)
{
  tug_cfg::BarConfig config;
  tug_cfg::RosParamReader reader(ros::NodeHandle("~bar1"));
  tug_cfg::load(config, reader);
  assertBar1LoadedCorrectly(config);
}

TEST(TestBarConfig, testLoadRosParam3)
{
  tug_cfg::BarConfig config;
  tug_cfg::RosParamReader reader(ros::NodeHandle("/test_ros_param_reader/bar1"));
  tug_cfg::load(config, reader);
  assertBar1LoadedCorrectly(config);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros_param_reader");
  return RUN_ALL_TESTS();
}
