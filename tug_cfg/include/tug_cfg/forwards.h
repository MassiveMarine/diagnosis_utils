#ifndef _TUG_CFG__FORWARDS_H_
#define _TUG_CFG__FORWARDS_H_

namespace tug_cfg
{
  class Collection;
  class AbstractMap;
  class AbstractScalar;
  class AbstractStruct;
  class AbstractSequence;
  class ConstVisitor;
  class DefaultConstrainer;
  class ExtensibleCollection;
  class Key;
  template <typename Key, typename ElementMetaT> class Map;
  class Object;
  template <typename T> class Scalar;
  template <typename C> class Struct;
  class Type;
  template <typename ElementMetaT> class Sequence;
  class Visitor;
}  // namespace tug_cfg

#endif  // _TUG_CFG__FORWARDS_H_
