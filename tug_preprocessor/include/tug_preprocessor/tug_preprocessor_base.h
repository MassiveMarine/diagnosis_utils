#ifndef TUG_PREPROCESSOR_BASE_H_
#define TUG_PREPROCESSOR_BASE_H_

#include <ros/ros.h>
#include <tug_plugin_manager/plugin_base.h>

namespace tug_preprocessor
{
class Preprocessor: public tug_plugin_manager::RegularPlugin
{
public:
  Preprocessor(){};
  virtual bool process(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
