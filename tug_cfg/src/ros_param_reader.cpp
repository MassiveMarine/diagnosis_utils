#include <tug_cfg/ros_param_reader.h>
#include <boost/lexical_cast.hpp>
#include <tug_cfg/collection.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>

namespace tug_cfg
{
RosParamReader::RosParamReader(const ros::NodeHandle& node_handle, const std::string& key)
  : top_(nullptr)
{
  if (!node_handle.getParam(key, root_node_))
  {
    // TODO: warn
  }
}

RosParamReader::RosParamReader(const XmlRpc::XmlRpcValue& value)
  : top_(nullptr), root_node_(value)
{
}

RosParamReader::~RosParamReader()
{
}

void RosParamReader::visit(Key* key, AbstractMap& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      for (; context.it != context.node->end(); ++context.it)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      // TODO: warn
      std::cerr << "Node is not struct" << std::endl;
    }
  }
}

void RosParamReader::visit(Key* key, AbstractSequence& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < context.node->size(); ++i)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      // TODO: warn
      std::cerr << "Node is not array" << std::endl;
    }
  }
}

void RosParamReader::visit(Key* key, AbstractStruct& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      value.acceptElements(key, *this);
      // TODO: find out whether all elements in map have been used
    }
    else
    {
      // TODO: warn
      std::cerr << "Node is not struct" << std::endl;
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<bool>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      value = *context.node;
    }
    else
    {
      // TODO: warn
      std::cerr << "Node is not bool" << std::endl;
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<double>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      value = *context.node;
    }
    else if (context.node->getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = static_cast<int>(*context.node);
    }
    else
    {
      // TODO: warn
      std::cerr << "Node is not double" << std::endl;
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<int>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = *context.node;
    }
    else
    {
      // TODO: warn
      std::cerr << "Node is not int" << std::endl;
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<std::string>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      value = *context.node;
    }
    else
    {
      // TODO: warn
      std::cerr << "Node is not string" << std::endl;
    }
  }
}

void RosParamReader::visit(Key* key, Object& value)
{
  // TODO: warn
  std::cerr << "Visiting object is not supported" << std::endl;
}



RosParamReader::Context::Context(RosParamReader* reader_)
  : reader(reader_), parent(reader_->top_), node(nullptr)
{
  reader->top_ = this;
}

RosParamReader::Context::~Context()
{
  reader->top_ = parent;
}

bool RosParamReader::Context::enter(Key* key)
{
  //std::cerr << "Entering " << key << std::endl;
  if (key == nullptr)
  {
    if (parent != nullptr)
    {
      throw std::logic_error("Null key");
    }
    node = &reader->root_node_;
  }
  else if (parent == nullptr)
  {
    throw std::logic_error("First key must be null");
  }
  else if (parent->node->getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    int* key_number = key->asPtr<int>();
    if (key_number != nullptr)
    {
      if (0 <= *key_number && *key_number < parent->node->size())
      {
        node = &(*parent->node)[*key_number];
      }
    }
    else
    {
      // TODO: warn
      std::cerr << "Sequence key is not int" << std::endl;
    }
  }
  else if (parent->node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
    if (field != nullptr)
    {
      node = &(*parent->node)[field->name];
    }
    else
    {
      std::string* key_name = key->asPtr<std::string>();
      if (key_name != nullptr)
      {
        *key_name = parent->it->first;
      }
      else
      {
        int* key_number = key->asPtr<int>();
        if (key_number != nullptr)
        {
          *key_number = boost::lexical_cast<int>(parent->it->first);
        }
        else
        {
          // TODO: warn
          std::cerr << "Map key is neither field, string, nor int" << std::endl;
        }
      }
      node = &parent->it->second;
    }
  }

  if (node != nullptr)
  {
    if (node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      it = node->begin();
    }
    return node->valid();
  }
  return false;
}
}  // namespace tug_cfg
