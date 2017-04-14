#include <tug_cfg/ros_dynamic_config_reader.h>
#include <algorithm>
#include <functional>
#include <ros/console.h>
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
  ROS_ERROR_NAMED("RosDynamicConfigReader", "unsupported type");
}

void RosDynamicConfigReader::visit(Key* key, AbstractSequence& value)
{
  ROS_ERROR_NAMED("RosDynamicConfigReader", "unsupported type");
}

void RosDynamicConfigReader::visit(Key* key, AbstractStruct& value)
{
  if (key == nullptr)
  {
    value.acceptElements(key, *this);
  }
  else
  {
    ROS_ERROR_NAMED("RosDynamicConfigReader", "unsupported nested struct");
  }
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
  if (field != nullptr)
  {
    for (const typename Values::value_type& p : values)
    {
      if (p.name == field->name)
      {
        value = static_cast<T>(p.value);  // Types do not always match
        break;
      }
    }
  }
}
}  // namespace tug_cfg
