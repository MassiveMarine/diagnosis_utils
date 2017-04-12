#include <tug_cfg/yaml_reader.h>
#include <exception>
#include <tug_cfg/collection.h>
#include <tug_cfg/key.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>
#include <yaml-cpp/yaml.h>

namespace tug_cfg
{
class YamlReader::Context
{
public:
  explicit Context(YamlReader* reader);
  ~Context();

  bool enter(Key* key);

  YAML::Node node;
  YAML::const_iterator it;

protected:
  YamlReader* reader_;
  Context* parent_;
};



class YamlReader::RootNode
{
public:
  RootNode(const YAML::Node& node_);

  YAML::Node node;
};



YamlReader::YamlReader(const std::string& file_path)
  : top_(nullptr)
{
  for (const YAML::Node& document : YAML::LoadAllFromFile(file_path))
  {
    root_nodes_.push(std::make_shared<RootNode>(document));
  }
}

void YamlReader::visit(Key* key, AbstractMap& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsMap())
    {
      for (; context.it != context.node.end(); ++context.it)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      // TODO: warn
    }
  }
}

void YamlReader::visit(Key* key, AbstractSequence& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsSequence())
    {
      for (; context.it != context.node.end(); ++context.it)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      // TODO: warn
    }
  }
}

void YamlReader::visit(Key* key, AbstractStruct& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsMap())
    {
      value.acceptElements(key, *this);
      // TODO: find out whether all elements in map have been used
    }
    else
    {
      // TODO: warn
    }
  }
}

void YamlReader::visit(Key* key, Scalar<bool>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<bool>();
    }
    else
    {
      // TODO: warn
    }
  }
}

void YamlReader::visit(Key* key, Scalar<double>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<double>();
    }
    else
    {
      // TODO: warn
    }
  }
}

void YamlReader::visit(Key* key, Scalar<int>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<int>();
    }
    else
    {
      // TODO: warn
    }
  }
}

void YamlReader::visit(Key* key, Scalar<std::string>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<std::string>();
    }
    else
    {
      // TODO: warn
    }
  }
}

void YamlReader::visit(Key* key, Object& value)
{
  // TODO: warn
}



YamlReader::Context::Context(YamlReader* reader)
  : reader_(reader), parent_(reader->top_)
{
  reader_->top_ = this;
}

YamlReader::Context::~Context()
{
  reader_->top_ = parent_;
}

bool YamlReader::Context::enter(Key* key)
{
  //std::cerr << "Entering " << key << std::endl;
  if (key == nullptr)
  {
    if (parent_ != nullptr)
    {
      throw std::logic_error("Null key");
    }
    if (!reader_->root_nodes_.empty())
    {
      node = reader_->root_nodes_.front()->node;
      reader_->root_nodes_.pop();
    }
  }
  else if (parent_ == nullptr)
  {
    throw std::logic_error("First key must be null");
  }
  else if (parent_->node.IsSequence())
  {
    node = static_cast<const YAML::Node&>(*parent_->it);
  }
  else if (parent_->node.IsMap())
  {
    const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
    if (field != nullptr)
    {
      node = parent_->node[field->name];
    }
    else
    {
      std::string* key_name = key->asPtr<std::string>();
      if (key_name != nullptr)
      {
        *key_name = parent_->it->first.as<std::string>();
      }
      else
      {
        int* key_number = key->asPtr<int>();
        if (key_number != nullptr)
        {
          *key_number = parent_->it->first.as<int>();
        }
        else
        {
          // TODO: warn
        }
      }
      node = parent_->it->second;
    }
  }
  it = node.begin();
  return node;
}

YamlReader::RootNode::RootNode(const YAML::Node& node_)
  : node(node_)
{
}
}  // namespace tug_cfg
