
#include "topology.hh"

#include <memory>
#include <ostream>

namespace Topo
{
struct ISaver
{
  virtual void save(const Object* _el) = 0;
  static std::shared_ptr<ISaver> make(std::ostream& _str);
};
}
