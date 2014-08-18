#ifndef TUG_PREPROCESSOR_BASE_H_
#define TUG_PREPROCESSOR_BASE_H_

#include <ros/ros.h>
#include <plugin_manager/plugin_base.h>

namespace plugin_base_preprocessor
{
class Preprocessor: public plugin_base::RegularPlugin
{
public:
  Preprocessor(){};
};
}

#endif
