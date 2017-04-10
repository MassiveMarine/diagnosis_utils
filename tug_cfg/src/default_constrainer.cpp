#include <tug_cfg/default_constrainer.h>
#include <tug_cfg/collection.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>
//#include <ros/console.h>
#include <iostream>

#define ROS_WARN_STREAM_NAMED(name, value) do { ((std::cerr << name << ": ") << value) << std::endl;  } while (false)

#define LOGGER_NAME "tug_cfg"

namespace tug_cfg
{
void DefaultConstrainer::visit(Key& key, AbstractMap& value)
{
  value.acceptElements(*this);
}

void DefaultConstrainer::visit(Key& key, Scalar<bool>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key& key, Scalar<double>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key& key, Scalar<int>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key& key, Scalar<std::string>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key& key, AbstractStruct& value)
{
  value.acceptElements(*this);
}

void DefaultConstrainer::visit(Key& key, AbstractSequence& value)
{
  value.acceptElements(*this);
}

void DefaultConstrainer::visit(Key& key, Object& value)
{
  ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration contains unknown type " << value.getType().getName());
}

template <typename T>
void DefaultConstrainer::constrainScalar(Key& key, Scalar<T>& value)
{
  AbstractStruct::Field* field = dynamic_cast<AbstractStruct::Field*>(&key);
  if (field != nullptr)
  {
    const AbstractStruct::ScalarFieldInfo<T>& info = dynamic_cast<const AbstractStruct::ScalarFieldInfo<T>&>(field->getInfo());
    enforceMin<T>(field->key, value, info.min);
    enforceMax<T>(field->key, value, info.max);
    enforceChoices<T>(field->key, value, info.choices, info.default_value);
  }
}

template <typename T>
void DefaultConstrainer::enforceMin(const std::string& name, T& value, const T& min_value)
{
  if (value < min_value)
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration parameter " << name << " was below specified minimum (" << value << " < " << min_value << ")");
    value = min_value;
  }
}

template <>
void DefaultConstrainer::enforceMin<std::string>(const std::string& name, std::string& value, const std::string& min_value)
{
  // String has no meaningful minimum, so we just ignore this.
}

template <typename T>
void DefaultConstrainer::enforceMax(const std::string& name, T& value, const T& max_value)
{
  if (value > max_value)
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration parameter " << name << " was above specified maximum (" << value << " > " << max_value << ")");
    value = max_value;
  }
}

template <>
void DefaultConstrainer::enforceMax<std::string>(const std::string& name, std::string& value, const std::string& max_value)
{
  // String has no meaningful maximum, so we just ignore this.
}

template <typename T>
void DefaultConstrainer::enforceChoices(const std::string& name, T& value, const std::set<T>& choices, const T& default_value)
{
  if (!choices.empty() && choices.find(value) == choices.end())
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration parameter " << name << " was not in set of valid choices");
    value = default_value;
  }
}
}  // namespace tug_cfg
