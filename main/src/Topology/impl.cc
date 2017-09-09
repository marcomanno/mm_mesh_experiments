
#include "impl.hh"
#include "persistence.hh"
#include <Utils/error_handling.hh>
#include <Utils/statistics.hh>
#include <Geo/vector.hh>
#include <Geo/iterate.hh>

namespace Topo {
//#pragma warning (disable : 4505)
template <Type typeT> 
size_t Base<typeT>::size(Direction _dir) const
{
  return _dir == Direction::Up ? up_elems_.size() : 0;
}

template <Type typeT>
IBase* Base<typeT>::get(Direction _dir, size_t _i) const
{
  if (_dir == Direction::Up && !up_elems_.empty())
    return up_elems_[_i % up_elems_.size()];
  else
    return nullptr;
}

template <Type typeT>
bool Base<typeT>::reversed(Direction /*_dir*/, size_t /*_i*/) const
{
  return false;
}

template <Type typeT>
bool Base<typeT>::replace(IBase* _new_elem)
{
  auto up_elems = up_elems_;
  for (const auto& prnt : up_elems)
    prnt->replace_child(this, _new_elem);
  return true;
}

template <Type typeT>
bool Base<typeT>::remove()
{
  auto up_elems = up_elems_;
  for (const auto& prnt : up_elems)
    prnt->remove_child(this);
  return true;
}

template <Type typeT>
size_t Base<typeT>::find_parent(const IBase* _prnt) const
{
  auto it = std::find(up_elems_.begin(), up_elems_.end(), _prnt);
  if (it == up_elems_.end())
    return SIZE_MAX;
  return it - up_elems_.begin();
}

template <Type typeT>
bool Base<typeT>::remove_parent(IBase* _prnt)
{
  auto it = std::find(up_elems_.begin(), up_elems_.end(), _prnt);
  if (it == up_elems_.end())
    return false;
  up_elems_.erase(it);
  return true;
}

template <Type typeT>
bool Base<typeT>::add_parent(IBase* _prnt)
{
  up_elems_.push_back(_prnt); 
  return true;
}

template <Type typeT>
UpEntity<typeT>::~UpEntity()
{
  for (size_t i = size(Direction::Down); i-- > 0;)
    remove_child(i);
}

template <Type typeT>
size_t UpEntity<typeT>::size(Direction _dir) const
{
  return _dir == Direction::Up ? up_elems_.size() : low_elems_.size();
}

template <Type typeT>
IBase* UpEntity<typeT>::get(Direction _dir, size_t _i) const
{
  if (_dir == Direction::Up)
    return up_elems_.empty() ? nullptr : up_elems_[_i % up_elems_.size()];
  else
    return low_elems_.empty() ? nullptr : low_elems_[_i % low_elems_.size()];
}

template <Type typeT>
bool UpEntity<typeT>::insert_child(IBase* _el, size_t _pos)
{
  if (_el == nullptr)
    return false;
  auto it = (_pos >= low_elems_.size()) ? low_elems_.end() : low_elems_.begin() + _pos;
  low_elems_.insert(it, _el);
  _el->add_ref();
  _el->add_parent(this);
  return true;
}

template <Type typeT>
bool UpEntity<typeT>::remove_child(size_t _pos)
{
  if (_pos >= low_elems_.size())
    return false;
  auto obj = low_elems_[_pos];
  low_elems_.erase(low_elems_.begin() + _pos);
  obj->remove_parent(this);
  obj->release_ref();
  return true;
}

template <Type typeT>
bool UpEntity<typeT>::remove_child(IBase* _el)
{
  return remove_child(find_child(_el));
}

template <Type typeT>
bool UpEntity<typeT>::replace_child(size_t _pos, IBase* _new_obj)
{
  if (_new_obj == nullptr)
    return false;
  if (low_elems_[_pos] == _new_obj)
    return true;

  _new_obj->add_ref();

  low_elems_[_pos]->remove_parent(this);
  low_elems_[_pos]->release_ref();

  low_elems_[_pos] = _new_obj;
  _new_obj->add_parent(this);
  return true;
}

template <Type typeT>
bool UpEntity<typeT>::replace_child(IBase* _el, IBase* _new_el)
{
  size_t pos = SIZE_MAX;
  for (bool replaced = false;;)
  {
    pos = find_child(_el, pos);
    if (pos == SIZE_MAX)
      return replaced;
    replaced |= replace_child(pos, _new_el);
  }
}

// search for an element in the range [0, _end[ in reverse order.
template <Type typeT>
size_t UpEntity<typeT>::find_child(const IBase* _el, size_t _end) const
{
  auto start_it = low_elems_.rbegin();
  if (_end < low_elems_.size())
    start_it += low_elems_.size() - _end;
  auto it = std::find(start_it, low_elems_.rend(), _el);
  if (it == low_elems_.rend())
    return SIZE_MAX;
  return low_elems_.rend() - it - 1;
}

template <Type typeT>
bool UpEntity<typeT>::remove()
{
  for (size_t i = size(Direction::Down); i-- > 0;)
    remove_child(i);
  return Base<typeT>::remove();
}

bool EE<Type::BODY>::insert_child(IBase* _el, size_t _pos)
{
  auto res = UpEntity<Type::BODY>::insert_child(_el, _pos);
  ordered_children_ &= !res;
  return res;
}

bool EE<Type::BODY>::replace_child(size_t _pos, IBase* _new_obj)
{
  auto res = UpEntity<Type::BODY>::replace_child(_pos, _new_obj);
  ordered_children_ &= !res;
  return res;
}

namespace {
bool compare_i_base(const IBase* _a, const IBase* _b) { return _a->id() < _b->id(); };
}//namespace

 // search for an element in the range [0, _end[ in reverse order.
size_t EE<Type::BODY>::find_child(const IBase* _el, size_t _end) const
{
  if (_end < low_elems_.size() || !ordered_children_)
    return UpEntity<Type::BODY>::find_child(_el, _end);

  auto it_pos = std::lower_bound(
    low_elems_.cbegin(), low_elems_.cend(), _el, compare_i_base);
  if (it_pos == low_elems_.end() || *it_pos != _el)
    return SIZE_MAX;

  return it_pos - low_elems_.begin();
}

void EE<Type::BODY>::optimize()
{
  if (ordered_children_)
    return;
  std::sort(low_elems_.begin(), low_elems_.end(), compare_i_base);
  ordered_children_ = true;
}

bool EE<Type::BODY>::remove()
{
  auto res = UpEntity<Type::BODY>::remove();
  ordered_children_ |= res;
  return res;
}

// Bodies have an optimized functions to remove childrens.
bool EE<Type::BODY>::remove_children(
  std::vector<IBase*>& _faces_to_remove)
{
  optimize();
  std::vector<IBase*> low_elems;

  std::set_difference(
    low_elems_.cbegin(), low_elems_.cend(), 
    _faces_to_remove.cbegin(), _faces_to_remove.cend(),
    std::back_inserter(low_elems), compare_i_base);
  low_elems_ = std::move(low_elems);
  return true;
}

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
  b.fatten(1.e-5);
  return b;
}

// A face can have all loops or all not loops.
bool EE<Type::FACE>::check()
{
  if (low_elems_.empty())
    return true;
  size_t loop_nmbr = 0;
  for (const auto el : low_elems_)
    loop_nmbr += el->type() == Type::LOOP;
  return loop_nmbr == 0 || loop_nmbr == low_elems_.size();
}

double EE<Type::VERTEX>::tolerance() const
{
  return std::max(tol_, Geo::epsilon(pt_));
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
  for (const auto& pt : seg)
    b += pt;
  b.fatten(tolerance());
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
  Vertices(const IBase* _loop, size_t _ind)
  {
    auto sz = _loop->size(Topo::Direction::Down);
    THROW_IF(_ind >= sz, "Bad toplogy access");
    auto add_vertex = [this, &_loop](size_t _i, size_t _ind)
    {
      auto vert = _loop->get(Topo::Direction::Down, _ind);
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

void CoEdgeRef::set_loop(IBase* _loop)
{
  loop_ = _loop;
}

bool CoEdgeRef::geom(Geo::Segment& _seg) const
{
  Vertices verts(loop_.get(), ind_);
  return verts.verts_[0]->geom(_seg[0]) && verts.verts_[1]->geom(_seg[1]);
}

double CoEdgeRef::tolerance() const
{
  Vertices verts(loop_.get(), ind_);
  return std::max(verts.verts_[0]->tolerance(), verts.verts_[1]->tolerance());
}

bool CoEdgeRef::operator<(const Object& _oth) const
{
  if (_oth.sub_type() != SubType::COEDGE_REF)
    return E<Type::COEDGE>::operator<(_oth);
  auto oth = static_cast<const CoEdgeRef&>(_oth);
  return loop_ < oth.loop_ || loop_ == oth.loop_ && ind_ < oth.ind_;
}

bool CoEdgeRef::operator==(const Object& _oth) const
{
  if (_oth.sub_type() != SubType::COEDGE_REF)
    return false;
  auto oth = static_cast<const CoEdgeRef&>(_oth);
  return loop_ == oth.loop_ && ind_ == oth.ind_;
}

IBase* LoopRef::loop() const { return loop_.get(); }
void LoopRef::set_loop(IBase* _loop) { loop_ = _loop; }
bool LoopRef::reverse() { return loop_->reverse(); }
Geo::Point LoopRef::internal_point() const { return loop_->internal_point(); }
Geo::Range<3> LoopRef::box() const { return loop_->box(); }

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
object_loader<SubType::EDGE>(std::istream&, ILoader*)
{
  return nullptr;
}

template <> void object_saver<SubType::EDGE_REF>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject 
object_loader<SubType::EDGE_REF>(std::istream&, ILoader*)
{
  return nullptr;
}

template <> void object_saver<SubType::COEDGE>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject
object_loader<SubType::COEDGE>(std::istream&, ILoader*)
{
  return nullptr;
}

template <> void object_saver<SubType::COEDGE_REF>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject
object_loader<SubType::COEDGE_REF>(std::istream&, ILoader*)
{
  return nullptr;
}

template <> void object_saver<SubType::LOOP>(
  std::ostream& _ostr, const Object* _obj, ISaver* _psav)
{
  save_base_entity<Type::LOOP>(
    _ostr, static_cast<const EE<Type::LOOP>*>(_obj), _psav);
}

template <> WrapObject
object_loader<SubType::LOOP>(std::istream& _istr, ILoader* _pload)
{
  Topo::Wrap<Type::LOOP> lp;
  load_base_entity(_istr, lp.make<EE<Type::LOOP>>(), _pload);
  return lp.get();
}

template <> void object_saver<SubType::LOOP_REF>(std::ostream&, const Object*, ISaver*)
{
}

template <> WrapObject
object_loader<SubType::LOOP_REF>(std::istream&, ILoader*)
{
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
