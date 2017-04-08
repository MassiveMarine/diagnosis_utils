#include <tug_cfg/configuration.h>
#include <tug_cfg/default_constrainer.h>

namespace tug_cfg
{
virtual void Configuration::constrain()
{
  constrain(DefaultConstrainer());
}

virtual void Configuration::load(Visitor& source)
{
  load(source, DefaultConstrainer());
}
}
