
#include "impl.hh"
#include "persistence.hh"
#include <Utils/error_handling.hh>
#include <Utils/statistics.hh>
#include <Geo/vector.hh>
#include <Geo/iterate.hh>

namespace Topo {

bool EE<Type::FACE>::reverse()
{
  std::reverse(low_elems_.begin(), low_elems_.end());
  return true;
}

Geo::Point EE<Type::FACE>::internal_point() const
{
  Geo::Point pt = {};
  if (!low_elems_.empty())
  {
    for (const auto el : low_elems_)
      pt += el->internal_point();
    pt /= double(low_elems_.size());
  }
  return pt;
}

Geo::Range<3> EE<Type::FACE>::box() const
{
  Utils::FindMax<double> max_tol;
  Geo::Range<3> b;
  for (const auto el : low_elems_)
  {
    max_tol.add(el->tolerance());
    auto pt = el->internal_point();
    max_tol.add(Geo::epsilon(pt));
    b += el->internal_point();
  }
  b.fatten(max_tol());
  return b;
}


bool EE<Type::EDGE>::geom(Geo::Segment& /*_seg*/) const
{
  return false;
}

bool EE<Type::EDGE>::set_geom(const Geo::Segment&)
{
  return false;
}

Geo::Range<3> EE<Type::VERTEX>::box() const
{
  Geo::Range<3> b;
  b += pt_;
  b.fatten(std::max(tol_, Geo::epsilon(pt_)));
  return b;
}


bool EdgeRef::geom(Geo::Segment& _seg) const
{
  return verts_[0]->geom(_seg[0]) && verts_[1]->geom(_seg[1]);
}

double EdgeRef::tolerance() const
{
  return std::max(verts_[0]->tolerance(), verts_[1]->tolerance());
}

Geo::Point EdgeRef::internal_point() const
{
  Geo::Segment seg;
  geom(seg);
  return (seg[0] + seg[1]) * 0.5;
}

Geo::Range<3> EdgeRef::box() const
{
  Geo::Range<3> b;
  Geo::Segment seg;
  geom(seg);
  Utils::FindMax<double> max_tol;
  for (const auto& pt : seg)
  {
    b += pt;
    max_tol.add(Geo::epsilon(pt));
  }
  max_tol.add(tolerance());
  b.fatten(max_tol());
  return b;
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
  const auto elem_nmbr = _base_ent->size(Direction::Down);
  _ostr << Utils::BinData<size_t>(elem_nmbr);
  for (size_t i = 0; i < elem_nmbr; ++i)
  {
    auto up_el = _base_ent->get(Direction::Down, i);
    _psav->save(up_el);
  }
}

void load_base_entity(std::istream& _istr, IBase* _base_ent, ILoader* _pload)
{
  size_t elem_nmbr;
  _istr >> Utils::BinData<size_t>(elem_nmbr);
  for (size_t i = 0; i < elem_nmbr; ++i)
  {
    auto obj = _pload->load();
    _base_ent->insert_child(static_cast<IBase*>(obj.get()));
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

template <> WrapObject
object_loader<SubType::VERTEX>(std::istream& _istr, ILoader* _pload)
{
  Topo::Wrap<Type::VERTEX> vert;
  load_base_entity(_istr, vert.make<EE<Type::VERTEX>>(), _pload);

  Geo::Point pt;
  _istr >> pt;
  vert->set_geom(pt);
  double tol;
  _istr >> Utils::BinData<double>(tol);
  vert->set_tolerance(tol);
  return WrapObject(vert.get());
}

template <> void object_saver<SubType::EDGE>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject 
object_loader<SubType::EDGE>(std::istream& _istr, ILoader* _pload)
{
  _istr; _pload;
  return nullptr;
}

template <> void object_saver<SubType::EDGE_REF>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject 
object_loader<SubType::EDGE_REF>(std::istream& _istr, ILoader* _pload)
{
  _istr; _pload;
  return nullptr;
}

template <> void object_saver<SubType::COEDGE>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject
object_loader<SubType::COEDGE>(std::istream& _istr, ILoader* _pload)
{
  _istr; _pload;
  return nullptr;
}

template <> void object_saver<SubType::COEDGE_REF>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject
object_loader<SubType::COEDGE_REF>(std::istream& _istr, ILoader* _pload)
{
  _istr; _pload;
  return nullptr;
}

template <> void object_saver<SubType::FACE>(
  std::ostream& _ostr, const Object* _obj, ISaver* _psav)
{
  save_base_entity<Type::FACE>(
    _ostr, static_cast<const EE<Type::FACE>*>(_obj), _psav);
}

template <> WrapObject
object_loader<SubType::FACE>(std::istream& _istr, ILoader* _pload)
{
  Topo::Wrap<Type::FACE> face;
  load_base_entity(_istr, face.make<EE<Type::FACE>>(), _pload);
  return face.get();
}

template <> void object_saver<SubType::BODY>(
  std::ostream& _ostr, const Object* _obj, ISaver* _psav)
{
  save_base_entity<Type::BODY>(
    _ostr, static_cast<const EE<Type::BODY>*>(_obj), _psav);
}

template <> WrapObject
object_loader<SubType::BODY>(std::istream& _istr, ILoader* _pload)
{
  Topo::Wrap<Type::BODY> body;
  load_base_entity(_istr, body.make<EE<Type::BODY>>(), _pload);
  return body.get();
}
}//namespace Topo
