#ifndef FIRST_POSTPROCESSOR_H_
#define FIRST_POSTPROCESSOR_H_

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>
#include <tug_postprocessor/tug_postprocessor_base.h>

namespace plugin_test_postprocessor
{

class FirstPostprocessor: public tug_postprocessor::Postprocessor
{
public:
  FirstPostprocessor();

  virtual void initialize(std::string name);
  virtual std::string getName();

  virtual bool process(const ros::Time& time, const ros::Duration& period);

private:
  std::string temp_data_;
};
}

#endif
