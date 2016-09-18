
#include "impl.hh"
#include "persistence.hh"
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

template<Type typeT>
void save_base_entity(std::ostream& _ostr, const Base<typeT>* _base_ent, ISaver* _psav)
{
  const auto elem_nmbr = _base_ent->size(Direction::Up);
  _ostr << Utils::BinData<size_t>(elem_nmbr);
  for (size_t i = 0; i < elem_nmbr; ++i)
  {
    auto up_el = _base_ent->get(Direction::Up, i);
    _psav->save(up_el);
  }
}

template <> void object_saver<SubType::VERTEX>(
  std::ostream& _ostr, const Object* _obj, ISaver* _psav)
{
  auto vert = static_cast<const EE<Type::VERTEX>*>(_obj);
  save_base_entity<Type::VERTEX>(_ostr, vert, _psav);
  Geo::Point pt;
  vert->geom(pt);
  _ostr << pt;
  _ostr << Utils::BinData<double>(vert->tolerance());
}

template <> void object_saver<SubType::EDGE>(std::ostream&, const Object*, ISaver*)
{
}

template <> void object_saver<SubType::EDGE_REF>(std::ostream&, const Object*, ISaver*)
{
}

template <> void object_saver<SubType::COEDGE>(std::ostream&, const Object*, ISaver*)
{
}

template <> void object_saver<SubType::COEDGE_REF>(std::ostream&, const Object*, ISaver*)
{
}

template <> void object_saver<SubType::FACE>(
  std::ostream& _ostr, const Object* _obj, ISaver* _psav)
{
  save_base_entity<Type::FACE>(
    _ostr, static_cast<const EE<Type::FACE>*>(_obj), _psav);
}

template <> void object_saver<SubType::BODY>(
  std::ostream& _ostr, const Object* _obj, ISaver* _psav)
{
  save_base_entity<Type::BODY>(
    _ostr, static_cast<const EE<Type::BODY>*>(_obj), _psav);
}

}//namespace Topo
