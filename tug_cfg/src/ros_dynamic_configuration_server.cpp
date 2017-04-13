#include <tug_cfg/ros_dynamic_configuration_server.h>

namespace tug_cfg
{
RosDynamicConfigurationServer::RosDynamicConfigurationServer()
  : configuration_(nullptr)
{
}

RosDynamicConfigurationServer::RosDynamicConfigurationServer(tug_cfg::AbstractStruct& configuration, const ros::NodeHandle& node_handle)
{
  start(configuration, node_handle);
}

RosDynamicConfigurationServer::~RosDynamicConfigurationServer()
{
  stop();
}

void RosDynamicConfigurationServer::start(tug_cfg::AbstractStruct& configuration, const ros::NodeHandle& node_handle)
{
  configuration_ = &configuration;

  // TODO: implement
}

void RosDynamicConfigurationServer::stop()
{
  configuration_ = nullptr;

  if (srv_)
  {
    srv_.shutdown();
  }

  if (descriptions_pub_)
  {
    descriptions_pub_.shutdown();
  }

  if (updates_pub_)
  {
    updates_pub_.shutdown();
  }
}
}  // namespace tug_cfg
