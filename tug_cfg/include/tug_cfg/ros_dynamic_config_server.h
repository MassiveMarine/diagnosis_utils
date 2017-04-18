#ifndef _TUG_CFG__ROS_DYNAMIC_CONFIG_SERVER_H_
#define _TUG_CFG__ROS_DYNAMIC_CONFIG_SERVER_H_

#include <dynamic_reconfigure/Reconfigure.h>
#include <functional>
#include <ros/node_handle.h>
#include <tug_cfg/struct.h>

namespace tug_cfg
{
class RosDynamicConfigServer
{
public:
  RosDynamicConfigServer();
  RosDynamicConfigServer(AbstractStruct& configuration);
  virtual ~RosDynamicConfigServer();

  void setConfiguration(AbstractStruct& configuration);
  void setCallback(const std::function<void()>& callback);
  void setLock(const std::function<void()>& lock, const std::function<void()>& unlock);

  template <typename Lockable>
  void setLock(Lockable& lockable)
  {
    setLock(std::bind(&Lockable::lock, &lockable),
            std::bind(&Lockable::unlock, &lockable));
  }

  void start(ros::NodeHandle node_handle=ros::NodeHandle("~"));
  void stop();

  /**
   * Publishes current state of configuration. Used to notify e.g. GUI of changes.
   */
  void notify();

protected:
  class ScopedLock
  {
  public:
    ScopedLock(const RosDynamicConfigServer* server);
    ~ScopedLock();

  protected:
    const RosDynamicConfigServer* server_;
  };
  friend class ScopedLock;

  RosDynamicConfigServer(const RosDynamicConfigServer&) = delete;
  RosDynamicConfigServer& operator=(const RosDynamicConfigServer&) = delete;

  bool reconfigure(dynamic_reconfigure::ReconfigureRequest& req,
                   dynamic_reconfigure::ReconfigureResponse& res);

  tug_cfg::AbstractStruct* configuration_;
  ros::ServiceServer srv_;
  ros::Publisher descriptions_pub_;
  ros::Publisher updates_pub_;
  std::function<void()> callback_;
  std::function<void()> lock_;
  std::function<void()> unlock_;
  bool reconfiguring_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__ROS_DYNAMIC_CONFIG_SERVER_H_
