#include <tug_cfg/ros_dynamic_config_server.h>
#include <boost/make_shared.hpp>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/ros_dynamic_config_reader.h>
#include <tug_cfg/ros_dynamic_config_describer.h>
#include <tug_cfg/ros_dynamic_config_writer.h>

namespace tug_cfg
{
RosDynamicConfigServer::RosDynamicConfigServer()
  : configuration_(nullptr), reconfiguring_(false)
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
  updates_pub_ = node_handle.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

  dynamic_reconfigure::ConfigDescriptionPtr description = boost::make_shared<dynamic_reconfigure::ConfigDescription>();
  RosDynamicConfigDescriber describer(description.get());
  store(*configuration_, describer);
  descriptions_pub_.publish(description);

  dynamic_reconfigure::ConfigPtr config = boost::make_shared<dynamic_reconfigure::Config>();
  RosDynamicConfigWriter writer(config.get());
  store(*configuration_, writer);
  updates_pub_.publish(config);
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

void RosDynamicConfigServer::notify()
{
  if (!reconfiguring_)
  {
    dynamic_reconfigure::ConfigPtr config = boost::make_shared<dynamic_reconfigure::Config>();
    RosDynamicConfigWriter writer(config.get());
    store(*configuration_, writer);
    updates_pub_.publish(config);
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
  ScopedLock lock(this);
  reconfiguring_ = true;
  try
  {
    RosDynamicConfigReader reader(&req.config);
    load(*configuration_, reader);
    if (callback_)
    {
      callback_();
    }
    RosDynamicConfigWriter writer(&res.config);
    store(*configuration_, writer);
    updates_pub_.publish(res.config);
  }
  catch (...)
  {
    reconfiguring_ = false;
    throw;
  }
  reconfiguring_ = false;
  return true;
}
}  // namespace tug_cfg
