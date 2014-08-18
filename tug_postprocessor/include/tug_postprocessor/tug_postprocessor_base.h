#ifndef TUG_POSTPORCESSOR_BASE_H_
#define TUG_POSTPROCESSOR_BASE_H_

#include <ros/ros.h>
#include <plugin_manager/plugin_base.h>

namespace plugin_base_postprocessor
{
class Postprocessor: public plugin_base::RegularPlugin
{
public:
  Postprocessor(){};
};
}

#endif
