#include "topology.hh"

namespace Topo
{

Object::Object() : ref_(0)
{
  static size_t progr_id;
  id_ = progr_id++;
}

Object::~Object()
{
}

bool Object::operator<(const Object& _oth) const { return id_ < _oth.id_; }
bool Object::operator==(const Object& _oth) const { return id_ == _oth.id_; }

}//namespace Topo