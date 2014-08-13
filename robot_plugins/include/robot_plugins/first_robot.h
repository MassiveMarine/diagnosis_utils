#ifndef ROBOT_PLUGINS_FIRST_ROBOT_BASE_H_
#define ROBOT_PLUGINS_FIRST_ROBOT_BASE_H_

#include <pluginlib/class_list_macros.h>
//#include <robot_plugins/plugin_base.h>
#include <plugin_manager/plugin_base.h>

namespace plugin_robot_ns
{

class FirstRobotLoigge: public plugin_base::RegularPlugin
{
public:
  FirstRobotLoigge()
  {
  }
  virtual void initialize(std::string name)
  {
    temp_data_ = name;
  }
  virtual std::string getName()
  {
    return temp_data_;
  }
private:
  std::string temp_data_;
};

//PLUGINLIB_EXPORT_CLASS(plugin_robot_ns::FirstRobotLoigge, plugin_base::RegularPlugin)

}
;
//namespace

#endif
