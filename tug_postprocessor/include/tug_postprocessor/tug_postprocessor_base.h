#ifndef TUG_POSTPORCESSOR_BASE_H_
#define TUG_POSTPROCESSOR_BASE_H_

#include <ros/ros.h>
#include <tug_plugin_manager/plugin_base.h>

namespace tug_postprocessor
{
class Postprocessor: public tug_plugin_manager::RegularPlugin
{
public:
  Postprocessor(){};
};
}

#endif
