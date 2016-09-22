
#include "topology.hh"

#include <memory>
#include <iostream>

namespace Topo
{
struct ISaver
{
  virtual void save(const Object* _el) = 0;
  static std::shared_ptr<ISaver> make(std::ostream& _str);
};

template <SubType> void object_saver(std::ostream&, const Object*, ISaver*);

struct ILoader
{
  virtual WrapObject load() = 0;
  static std::shared_ptr<ILoader> make(std::istream& _str);
};

template <SubType> Topo::WrapObject
  object_loader(std::istream&, ILoader*);
}
