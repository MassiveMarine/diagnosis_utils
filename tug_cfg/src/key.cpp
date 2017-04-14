#include <tug_cfg/key.h>

namespace tug_cfg
{

Key::Key(const Key* parent)
  : parent_(parent)
{
}

const Key* Key::getParent() const
{
  return parent_;
}

void Key::formatPath(std::ostream& s) const
{
  if (parent_ != nullptr)
  {
    parent_->formatPath(s);
  }
  format(s);
}

std::ostream& operator<<(std::ostream& s, const Key& key)
{
  s << &key;
}

std::ostream& operator<<(std::ostream& s, const Key* key)
{
  if (key != nullptr)
  {
    key->formatPath(s);
  }
  return s;
}

}
