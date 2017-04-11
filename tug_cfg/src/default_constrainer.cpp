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
void DefaultConstrainer::visit(Key* key, AbstractMap& value)
{
  value.acceptElements(key, *this);
}

void DefaultConstrainer::visit(Key* key, Scalar<bool>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, Scalar<double>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, Scalar<int>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, Scalar<std::string>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, AbstractStruct& value)
{
  value.acceptElements(key, *this);
}

void DefaultConstrainer::visit(Key* key, AbstractSequence& value)
{
  value.acceptElements(key, *this);
}

void DefaultConstrainer::visit(Key* key, Object& value)
{
  ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration contains unknown type " << value.getType().getName() << " at " << key);
}

template <typename T>
void DefaultConstrainer::constrainScalar(Key* key, Scalar<T>& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr)
  {
    const AbstractStruct::ScalarField<T>& scalar_field = dynamic_cast<const AbstractStruct::ScalarField<T>&>(*field);
    enforceMin<T>(key, value, scalar_field.min);
    enforceMax<T>(key, value, scalar_field.max);
    enforceChoices<T>(key, value, scalar_field.choices, scalar_field.default_value);
  }
}

template <typename T>
void DefaultConstrainer::enforceMin(Key* key, T& value, const T& min_value)
{
  if (value < min_value)
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration parameter " << key << " was below specified minimum (" << value << " < " << min_value << ")");
    value = min_value;
  }
}

template <>
void DefaultConstrainer::enforceMin<std::string>(Key* key, std::string& value, const std::string& min_value)
{
  // String has no meaningful minimum, so we just ignore this.
}

template <typename T>
void DefaultConstrainer::enforceMax(Key* key, T& value, const T& max_value)
{
  if (value > max_value)
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration parameter " << key << " was above specified maximum (" << value << " > " << max_value << ")");
    value = max_value;
  }
}

template <>
void DefaultConstrainer::enforceMax<std::string>(Key* key, std::string& value, const std::string& max_value)
{
  // String has no meaningful maximum, so we just ignore this.
}

template <typename T>
void DefaultConstrainer::enforceChoices(Key* key, T& value, const std::set<T>& choices, const T& default_value)
{
  if (!choices.empty() && choices.find(value) == choices.end())
  {
    ROS_WARN_STREAM_NAMED(LOGGER_NAME, "Configuration parameter " << key << " was not in set of valid choices");
    value = default_value;
  }
}
}  // namespace tug_cfg
