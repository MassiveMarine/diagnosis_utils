#include <tug_cfg/ros_dynamic_config_server.h>
#include <boost/make_shared.hpp>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/ros_dynamic_config_reader.h>
#include <tug_cfg/ros_dynamic_config_describer.h>

namespace tug_cfg
{
RosDynamicConfigServer::RosDynamicConfigServer()
  : configuration_(nullptr)
{
}

RosDynamicConfigServer::RosDynamicConfigServer(AbstractStruct& configuration)
  : configuration_(&configuration)
{
}

RosDynamicConfigServer::~RosDynamicConfigServer()
{
  stop();
}

void RosDynamicConfigServer::setConfiguration(AbstractStruct& configuration)
{
  configuration_ = &configuration;
}

void RosDynamicConfigServer::setCallback(const std::function<void()>& callback)
{
  callback_ = callback;
}

void RosDynamicConfigServer::setLock(const std::function<void()>& lock, const std::function<void()>& unlock)
{
  lock_ = lock;
  unlock_ = unlock;
}

void RosDynamicConfigServer::start(ros::NodeHandle node_handle)
{
  // Note: node_handle must not be const because advertiseService is non-const
  srv_ = node_handle.advertiseService("set_parameters", &RosDynamicConfigServer::reconfigure, this);
  descriptions_pub_ = node_handle.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
  updates_pub_ = node_handle.advertise<dynamic_reconfigure::Config>("parameter_updates", 1);

  dynamic_reconfigure::ConfigDescriptionPtr description = boost::make_shared<dynamic_reconfigure::ConfigDescription>();
  RosDynamicConfigDescriber describer(description.get());
  configuration_->accept(nullptr, describer);
  descriptions_pub_.publish(description);
}

void RosDynamicConfigServer::stop()
{
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

RosDynamicConfigServer::ScopedLock::ScopedLock(const RosDynamicConfigServer* server)
  : server_(server)
{
  if (server_->lock_)
  {
    server_->lock_();
  }
}

RosDynamicConfigServer::ScopedLock::~ScopedLock()
{
  if (server_->unlock_)
  {
    server_->unlock_();
  }
}

bool RosDynamicConfigServer::reconfigure(
    dynamic_reconfigure::ReconfigureRequest& req,
    dynamic_reconfigure::ReconfigureResponse& res)
{
  RosDynamicConfigReader reader(&req.config);
  load(*configuration_, reader);
  if (callback_)
  {
    callback_();
  }
  res.config = req.config;
  updates_pub_.publish(res.config);
  return true;
}
}  // namespace tug_cfg
