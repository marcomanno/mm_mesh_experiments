
#include "topology.hh"

#include <memory>

namespace Topo
{
struct ISaver
{
  virtual void add_element(Object* _el) = 0;
  virtual bool save() = 0;
  static std::shared_ptr<ISaver> make();
};
}
