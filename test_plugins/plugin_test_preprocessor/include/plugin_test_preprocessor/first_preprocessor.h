#ifndef FIRST_PREPROCESSOR_H_
#define FIRST_PREPROCESSOR_H_

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>
#include <tug_preprocessor/tug_preprocessor_base.h>

namespace plugin_test_preprocessor
{

class FirstPreprocessor: public tug_preprocessor::Preprocessor
{
public:
  FirstPreprocessor();

  virtual void initialize(std::string name);
  virtual std::string getName();

  virtual bool process(const ros::Time& time, const ros::Duration& period);

private:
  std::string temp_data_;
};
}

#endif
