
#include "impl.hh"
#include "saver.hh"
#include <Utils/error_handling.hh>
#include <Utils/statistics.hh>
#include <Geo/vector.hh>

namespace Topo {

bool EE<Type::EDGE>::geom(Geo::Segment& /*_seg*/) const
{
  return false;
}

bool EE<Type::EDGE>::set_geom(const Geo::Segment&)
{
  return false;
}

bool EdgeRef::geom(Geo::Segment& _seg) const
{
  return verts_[0]->geom(_seg[0]) && verts_[1]->geom(_seg[1]);
}

double EdgeRef::tolerance() const
{
  return std::max(verts_[0]->tolerance(), verts_[1]->tolerance());
}

bool EdgeRef::operator<(const Object& _oth) const
{
  if (_oth.sub_type() != SubType::EDGE_REF)
    return E<Type::EDGE>::operator<(_oth);
  auto oth = static_cast<const EdgeRef&>(_oth);
  return verts_[0] < oth.verts_[0] ||
    (verts_[0] == oth.verts_[0] && verts_[1] < oth.verts_[1]);
}

bool EdgeRef::operator==(const Object& _oth) const
{
  if (_oth.sub_type() != SubType::EDGE_REF)
    return E<Type::EDGE>::operator==(_oth);
  auto oth = static_cast<const EdgeRef&>(_oth);
  return verts_[0] == oth.verts_[0] && verts_[1] == oth.verts_[1];
}

void EdgeRef::finalise()
{
  if (verts_[1] < verts_[0])
    std::swap(verts_[0], verts_[1]);
}

namespace {

struct Vertices
{
  Vertices(const Wrap<Type::FACE>& _face, size_t _ind)
  {
    auto sz = _face->size(Topo::Direction::Down);
    THROW_IF(_ind >= sz, "Bad toplogy access");
    auto add_vertex = [this, &_face](size_t _i, size_t _ind)
    {
      auto vert = _face->get(Topo::Direction::Down, _ind);
      THROW_IF(vert->type() != Topo::Type::VERTEX, "Unexpected type");
      verts_[_i].reset(static_cast<E<Type::VERTEX> *>(vert));
    };
    add_vertex(0, _ind);
    if (++_ind >= sz)
      _ind = 0;
    add_vertex(1, _ind);
  }
  Wrap<Type::VERTEX> verts_[2];
};
}//namespace
  
bool CoEdgeRef::geom(Geo::Segment& _seg) const
{
  Vertices verts(face_, ind_);
  return verts.verts_[0]->geom(_seg[0]) && verts.verts_[1]->geom(_seg[1]);
}

double CoEdgeRef::tolerance() const
{
  Vertices verts(face_, ind_);
  return std::max(verts.verts_[0]->tolerance(), verts.verts_[1]->tolerance());
}

bool CoEdgeRef::operator<(const Object& _oth) const
{
  if (_oth.sub_type() != SubType::COEDGE_REF)
    return E<Type::COEDGE>::operator<(_oth);
  auto oth = static_cast<const CoEdgeRef&>(_oth);
  return face_ < oth.face_ || face_ == oth.face_ && ind_ < oth.ind_;
}

bool CoEdgeRef::operator==(const Object& _oth) const
{
  if (_oth.sub_type() != SubType::COEDGE_REF)
    return false;
  auto oth = static_cast<const CoEdgeRef&>(_oth);
  return face_ == oth.face_ && ind_ == oth.ind_;
}

void saveUpEntity(std::ostream& _ostr,
  const EE<Type::VERTEX>* _vert, ISaver* _psav)
{
  _ostr; _psav; _vert;
}



template <> void object_saver<SubType::VERTEX>(
  std::ostream& _ostr, const Object* _obj, ISaver* _psav)
{
  auto vert = static_cast<const EE<Type::VERTEX>*>(_obj);
  saveUpEntity(_ostr, vert, _psav);
  Geo::Point pt;
  vert->geom(pt);
  _ostr << int(SubType::VERTEX);
  _ostr << pt;
  _ostr << Utils::BinData<double>(vert->tolerance());
}

template <> void object_saver<SubType::EDGE>(std::ostream&, const Object*, ISaver*)
{

}

template <> void object_saver<SubType::EDGE_REF>(std::ostream&, const Object*, ISaver*)
{

}

template <> void object_saver<SubType::COEDGE_REF>(std::ostream&, const Object*, ISaver*)
{

}

template <> void object_saver<SubType::FACE>(std::ostream&, const Object*, ISaver*)
{

}

template <> void object_saver<SubType::BODY>(std::ostream&, const Object*, ISaver*)
{

}

}//namespace Topo
