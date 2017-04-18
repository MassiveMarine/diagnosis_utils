#include <tug_cfg/ros_dynamic_config_reader.h>
#include <algorithm>
#include <functional>
#include <ros/console.h>
#include <sstream>
#include <tug_cfg/key.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>

namespace tug_cfg
{
RosDynamicConfigReader::RosDynamicConfigReader(const dynamic_reconfigure::Config* config)
  : config_(config)
{
}

RosDynamicConfigReader::~RosDynamicConfigReader()
{
}

void RosDynamicConfigReader::visit(Key* key, AbstractMap& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ROS_WARN_NAMED("RosDynamicConfigReader", "tried to read map");
  }
}

void RosDynamicConfigReader::visit(Key* key, AbstractSequence& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ROS_WARN_NAMED("RosDynamicConfigReader", "tried to read sequence");
  }
}

void RosDynamicConfigReader::visit(Key* key, AbstractStruct& value)
{
  value.acceptElements(key, *this);
}

void RosDynamicConfigReader::visit(Key* key, Scalar<bool>& value)
{
  visitScalar(key, value, config_->bools);
}

void RosDynamicConfigReader::visit(Key* key, Scalar<double>& value)
{
  visitScalar(key, value, config_->doubles);
}

void RosDynamicConfigReader::visit(Key* key, Scalar<int>& value)
{
  visitScalar(key, value, config_->ints);
}

void RosDynamicConfigReader::visit(Key* key, Scalar<std::string>& value)
{
  visitScalar(key, value, config_->strs);
}

void RosDynamicConfigReader::visit(Key* key, Object& value)
{
  ROS_ERROR_NAMED("RosDynamicConfigReader", "unsupported type");
}

template <typename T, typename Values>
void RosDynamicConfigReader::visitScalar(Key* key, Scalar<T>& value,
                                         const Values& values)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    std::ostringstream s;
    s << key;
    std::string name(s.str());
    for (const typename Values::value_type& p : values)
    {
      if (p.name == name)
      {
        value = static_cast<T>(p.value);  // Types do not always match
        break;
      }
    }
  }
}
}  // namespace tug_cfg
