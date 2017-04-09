#ifndef _TUG_CFG__CONFIGURATION_H_
#define _TUG_CFG__CONFIGURATION_H_

#include <tug_cfg/forwards.h>

namespace tug_cfg
{
void constrain(Object& value);
void constrain(Object& value, Visitor& constrainer);
void load(Object& value, Visitor& source);
void load(Object& value, Visitor& source, Visitor& constrainer);
void store(const Object& value, ConstVisitor& sink);
}

#endif
