
#include "impl.hh"
#include "iterator.hh"
#include "subtype.hh"

#include "Utils/error_handling.hh"

#include <map>
#include <vector>
#include <set>

namespace Topo {

template <Type FromT, Type ToT> 
Iterator<FromT, ToT>::Iterator() : impl_(new Impl)
{
}

template <Type FromT, Type ToT>
void Iterator<FromT, ToT>::reset(const Wrap<FromT>& _from)
{
  impl_->reset(_from);
}

template <Type FromT, Type ToT>
Iterator<FromT, ToT>::~Iterator()
{
  delete impl_;
}

template <Type FromT, Type ToT>
size_t Iterator<FromT, ToT>::size() const
{
  return impl_->size();
}

template <Type FromT, Type ToT>
Wrap<ToT> Iterator<FromT, ToT>::get(size_t _i) const { return impl_->get(_i); }

template <Type FromT, Type ToT>
Wrap<ToT>* Iterator<FromT, ToT>::begin() const { return impl_->begin(); }

template <Type FromT, Type ToT>
Wrap<ToT>* Iterator<FromT, ToT>::end() const { return impl_->end(); }


template <Type FromT, Type ToT> 
void Iterator<FromT, ToT>::clear() { return impl_->clear(); }

namespace {

template <Type> struct TopoSubtype {};

#define SUBTYPE_RELATION(TopoType) template <> struct TopoSubtype<Type::TopoType> \
{ static const SubType Value = SubType::TopoType; };

SUBTYPE_RELATION(BODY)
SUBTYPE_RELATION(FACE)
SUBTYPE_RELATION(LOOP)
SUBTYPE_RELATION(COEDGE)
SUBTYPE_RELATION(EDGE)
SUBTYPE_RELATION(VERTEX)

template <Type typeT> struct BodyIteratorBase
{
  size_t size() const { return elems_.size(); }
  Wrap<typeT> get(size_t _i) const { return elems_[_i]; }
  Wrap<typeT>* begin() { return elems_.data(); }
  Wrap<typeT>* end() { return elems_.data() + elems_.size(); }
  void clear() { elems_.clear();  }
  std::vector<Wrap<typeT>> elems_;
};

struct IterElement
{
  virtual bool add(IBase*) { return true; };
  virtual bool process(IBase*, size_t) { return true; };
};

template <Topo::Type typeT>
struct AddIterElement : public IterElement
{
  std::vector<Wrap<typeT>>& elems_;
  AddIterElement(std::vector<Wrap<typeT>>& _elems) : elems_(_elems) {}
  virtual bool add(IBase* _el) override
  {
    elems_.push_back(static_cast<E<typeT>*>(_el));
    return true;
  }
};

template <Topo::Type typeT>
struct AddIterUniqueElement : public AddIterElement<typeT>
{
  using AddIterElement::AddIterElement;
  std::set<Wrap<typeT>> elems_set_;
  virtual bool add(IBase* _el) override
  {
    if (elems_set_.insert(static_cast<E<typeT>*>(_el)).second)
      elems_.push_back(static_cast<E<typeT>*>(_el));
    return true;
  }
};

template <Type from_typeT, Type to_typeT> struct AddElementSelector
{ typedef AddIterElement<to_typeT> Value; };

template <> struct AddElementSelector<Type::BODY, Type::VERTEX>
{ typedef AddIterUniqueElement<Type::VERTEX> Value; };

template <Direction dirT, Type to_typeT>
bool topo_iterate(IBase* _from, IterElement& _op)
{
  if (_from->type() == to_typeT)
    return _op.add(_from);
  if ((dirT == Direction::Down) != _from->type() > to_typeT)
    return false;
  auto elm_nmbr = _from->size(dirT);
  for (size_t i = 0; i < elm_nmbr; ++i)
    if (!topo_iterate<dirT, to_typeT>(_from->get(dirT, i), _op))
      _op.process(_from, i);
  return true;
}

template <Type from_typeT, Type to_typeT>
void get_elements(const Wrap<from_typeT>& _from, std::vector<Wrap<to_typeT>>& _elems)
{
  AddElementSelector<from_typeT, to_typeT>::Value add_elems(_elems);
  const Direction dir = from_typeT > to_typeT ? Direction::Down : Direction::Up;
  topo_iterate<dir, to_typeT>(
    const_cast<IBase*>(static_cast<const IBase*>(_from.get())), add_elems);
}

bool base_ptr_less(const IBase* _a, const IBase* _b)
{
  if (_a == _b)
    return false;
  if (_a == nullptr)
    return true;
  if (_b == nullptr)
    return false;
  return _a->id() < _b->id();
}

}//namespace

template <Type from_typeT, Type to_typeT>
struct Iterator<from_typeT, to_typeT>::Impl : public BodyIteratorBase<to_typeT>
{
  void reset(const Wrap<from_typeT>& _from)
  {
    clear();
    get_elements(_from, elems_);
  }
};

template <Type FromT>
struct BodyIteratorToEdge : public BodyIteratorBase<Type::EDGE>
{
  void reset(const Wrap<FromT>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != TopoSubtype<FromT>::Value, "Wrong type");

    struct AddIterEdge : public IterElement
    {
      std::vector<Wrap<Type::EDGE>>& elems_;
      AddIterEdge(std::vector<Wrap<Type::EDGE>>& _elems) : elems_(_elems) {}
      ~AddIterEdge()
      {
        std::sort(elems_.begin(), elems_.end());
        auto new_size = std::unique(elems_.begin(), elems_.end()) - elems_.begin();
        elems_.resize(new_size);
      }
      virtual bool process(IBase* _from, size_t _i)
      {
        Wrap<Type::EDGE> edg_wrp;
        auto edg = edg_wrp.make<EdgeRef>();
        edg->verts_[0] = static_cast<E<Type::VERTEX>*>(_from->get(Direction::Down, _i));
        if (++_i >= _from->size(Direction::Down)) _i = 0;
        edg->verts_[1] = static_cast<E<Type::VERTEX>*>(_from->get(Direction::Down, _i));
        edg->finalise();
        elems_.push_back(edg);
        return true;
      }
    };
    AddIterEdge edge_adder(elems_);
    topo_iterate<Direction::Down, Type::EDGE>(
      const_cast<IBase*>(static_cast<const IBase*>(_from.get())), edge_adder);
  }
};

template <>
struct Iterator<Type::BODY, Type::EDGE>::Impl : public BodyIteratorToEdge<Type::BODY>
{};

template <>
struct Iterator<Type::EDGE, Type::VERTEX>::Impl : public BodyIteratorBase<Type::VERTEX>
{
  void reset(const Wrap<Type::EDGE>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != SubType::EDGE_REF, "Expected EDGE_REF");
    auto ed_ref = static_cast<const EdgeRef*>(_from.get());
    for (size_t i = 0; i < std::size(ed_ref->verts_); ++i)
      elems_.push_back(ed_ref->verts_[i]);
  }
};

template <>
struct Iterator<Type::VERTEX, Type::EDGE>::Impl : public BodyIteratorBase<Type::EDGE>
{
  void reset(const Wrap<Type::VERTEX>& _from)
  {
    clear();

    std::set<Wrap<Type::VERTEX>> already_used;
    for (size_t i = 0; i < _from->size(Direction::Up); ++i)
    {
      auto face = _from->get(Direction::Up, i);
      THROW_IF(face->type() != Type::FACE && face->type() != Type::LOOP,
        "Not expectedtype");
      for (auto pos = face->find_child(_from.get()); pos != SIZE_MAX; 
        pos = face->find_child(_from.get(), pos))
      {
        size_t pos_near[2] = {pos , pos + 1 };
        if (pos_near[0] > 0) --pos_near[0];
        else pos_near[0] = face->size(Direction::Down) - 1;
        if (pos_near[1] >= face->size(Direction::Down)) pos_near[1] = 0;

        for (auto pos_oth : pos_near)
        {
          auto vert_oth = face->get(Direction::Down, pos_oth);
          if (vert_oth->type() != Type::VERTEX)
            continue;
          Wrap<Type::VERTEX> vert(static_cast<E<Type::VERTEX>*>(vert_oth));
          if (!already_used.insert(vert).second)
            continue;
          Wrap<Type::EDGE> edge;
          auto edref = edge.make<EdgeRef>();
          edref->verts_[0] = _from;
          edref->verts_[1].reset(static_cast<E<Type::VERTEX>*>(vert.get()));
          edref->finalise();
          elems_.emplace_back(edge);
        }
      }
    }
  }
};

namespace {

template <class OperationT>
void find_coedges(const Wrap<Type::EDGE>& _from, OperationT& _op)
{
  THROW_IF(_from->sub_type() != SubType::EDGE_REF, "Not expected edge type");
  auto edge_ref = static_cast<const Topo::EdgeRef*>(_from.get());
  std::vector<IBase*> faces[2], common_faces;
  for (int i = 0; i < 2; ++i)
  {
    auto& vert = edge_ref->verts_[i];
    for (size_t j = 0; j < vert->size(Topo::Direction::Up); ++j)
    {
      auto ptr_face = vert->get(Topo::Direction::Up, j);
      THROW_IF(ptr_face->type() != Type::FACE && ptr_face->type() != Type::LOOP,
        "Unexpected type");
      faces[i].emplace_back(ptr_face);
    }
    std::sort(faces[i].begin(), faces[i].end(), base_ptr_less);
    faces[i].erase(std::unique(faces[i].begin(), faces[i].end()), faces[i].end());
  }
  std::set_intersection(
    faces[0].begin(), faces[0].end(),
    faces[1].begin(), faces[1].end(),
    std::back_inserter(common_faces),
    base_ptr_less);

  for (auto parent_el : common_faces)
  {
    auto vert_ptr = edge_ref->verts_[0].get();
    for (auto pos = parent_el->find_child(vert_ptr); pos != SIZE_MAX;
      pos = parent_el->find_child(vert_ptr, pos))
    {
      size_t pos_near[2] = { pos , pos + 1 };
      if (pos_near[0] > 0) --pos_near[0];
      else pos_near[0] = parent_el->size(Direction::Down) - 1;
      if (pos_near[1] >= parent_el->size(Direction::Down)) pos_near[1] = 0;

      for (int i = 0; i < 2; ++i)
      {
        auto pos_oth = pos_near[i];
        auto vert_oth = parent_el->get(Direction::Down, pos_oth);
        if (vert_oth->type() != Type::VERTEX)
          continue;
        if (vert_oth != edge_ref->verts_[1].get())
          continue;
        _op(parent_el, i > 0 ? pos : pos_near[0]);
      }
    }
  }
}
} // namespace

template <>
struct Iterator<Type::EDGE, Type::COEDGE>::Impl : public BodyIteratorBase<Type::COEDGE>
{
  void reset(const Wrap<Type::EDGE>& _from)
  {
    clear();
    struct CoedgeAdder
    {
      std::vector<Wrap<Type::COEDGE>>& elems_;
      CoedgeAdder(std::vector<Wrap<Type::COEDGE>>& _elems) : elems_(_elems) {}
      void operator()(IBase* _parent, size_t _pos)
      {
        Wrap<Type::COEDGE> coedge;
        auto cedge_data = coedge.make<CoEdgeRef>();
        cedge_data->set_loop(_parent);
        cedge_data->ind_ = _pos;
        elems_.emplace_back(coedge);
      }
    } coedge_adder(elems_);
    find_coedges(_from, coedge_adder);
  }
};

template <>
struct Iterator<Type::EDGE, Type::FACE>::Impl : public BodyIteratorBase<Type::FACE>
{
  void reset(const Wrap<Type::EDGE>& _from)
  {
    clear();
    struct FaceAdder
    {
      std::vector<Wrap<Type::FACE>>& elems_;
      FaceAdder(std::vector<Wrap<Type::FACE>>& _elems) : elems_(_elems) {}
      void operator()(IBase* _parent, size_t)
      {
        if (_parent->type() == Type::FACE)
          elems_.emplace_back(static_cast<E<Type::FACE>*>(_parent));
        else if (_parent->type() == Type::LOOP)
        {
          for (rsize_t i = 0; i < _parent->size(Direction::Up); ++i)
          {
            auto face = _parent->get(Direction::Up, i);
            if (face->type() == Type::FACE)
              elems_.emplace_back(static_cast<E<Type::FACE>*>(face));
          }
        }
      }
    } face_adder(elems_);
    find_coedges(_from, face_adder);
  }
};

template <>
struct Iterator<Type::COEDGE, Type::FACE>::Impl : public BodyIteratorBase<Type::FACE>
{
  void reset(const Wrap<Type::COEDGE>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != SubType::COEDGE_REF, "Not expected coedge type");
    auto coedge_ref = static_cast<const Topo::CoEdgeRef*>(_from.get());
    auto loop_or_face = coedge_ref->loop();
    if (loop_or_face->type() == Type::FACE)
      elems_.emplace_back(Wrap<Type::FACE>(static_cast<E<Type::FACE>*>(loop_or_face)));
    else
      for (size_t i = 0; i < loop_or_face->size(Direction::Up); ++i)
      {
        auto face = loop_or_face->get(Direction::Up, i);
        THROW_IF(face->type() != Type::FACE, "Expected face");
        elems_.emplace_back(Wrap<Type::FACE>(static_cast<E<Type::FACE>*>(face)));
      }
  }
};

template <>
struct Iterator<Type::COEDGE, Type::EDGE>::Impl : public BodyIteratorBase<Type::EDGE>
{
  void reset(const Wrap<Type::COEDGE>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != SubType::COEDGE_REF, "Not expected coedge type");
    auto coedge_ref = static_cast<const Topo::CoEdgeRef*>(_from.get());
    auto second = coedge_ref->ind_ + 1;
    if (second > coedge_ref->loop()->size(Direction::Down))
      second = 0;

    Wrap<Type::EDGE> edge;
    auto edref = edge.make<EdgeRef>();
    auto v0 = static_cast<E<Type::VERTEX>*>(
      coedge_ref->loop()->get(Direction::Down, coedge_ref->ind_));
    edref->verts_[0].reset(v0);
    auto v1 = static_cast<E<Type::VERTEX>*>(coedge_ref->loop()->get(Direction::Down, second));
    edref->verts_[1].reset(v1);
    edref->finalise();
    elems_.emplace_back(edge);
  }
};

template <>
struct Iterator<Type::FACE, Type::EDGE>::Impl : public BodyIteratorToEdge<Type::FACE>
{};

namespace {

template <Type FromT>
struct IterToCoedge : public BodyIteratorBase<Type::COEDGE>
{
  virtual IBase* get_parent(const Wrap<FromT>& _from)
  {
    THROW_IF(_from->sub_type() != TopoSubtype<FromT>::Value, "Wrong type");
    return _from.get();
  }
  void reset(const Wrap<FromT>& _from)
  {
    clear();
    IBase* parent = get_parent(_from);
    struct CoedgeAdder : public IterElement
    {
      std::vector<Wrap<Type::COEDGE>>& elems_;
      CoedgeAdder(std::vector<Wrap<Type::COEDGE>>& _elems) : elems_(_elems) {}
      virtual bool process(IBase* _parent, size_t _i)
      {
        Wrap<Type::COEDGE> co_edge;
        auto co_edref = co_edge.make<CoEdgeRef>();
        co_edref->set_loop(_parent);
        co_edref->ind_ = _i;
        elems_.emplace_back(co_edge);
        return true;
      };
    };
    CoedgeAdder coedge_adder(elems_);
    topo_iterate<Direction::Down, Type::COEDGE>(parent, coedge_adder);
  }
};

} // namespace

template <>
struct Iterator<Type::BODY, Type::COEDGE>::Impl : public IterToCoedge<Type::BODY> {};

template <>
struct Iterator<Type::FACE, Type::COEDGE>::Impl : public IterToCoedge<Type::FACE> {};

template <>
struct Iterator<Type::LOOP, Type::COEDGE>::Impl : public IterToCoedge<Type::LOOP>
{
  virtual IBase* get_parent(const Wrap<Type::LOOP>& _from) override
  {
    if (_from->sub_type() == SubType::LOOP_REF)
      return static_cast<LoopRef*>(_from.get())->loop();
    return IterToCoedge<Type::LOOP>::get_parent(_from);
  }
};

template <>
struct Iterator<Type::COEDGE, Type::VERTEX>::Impl : public BodyIteratorBase<Type::VERTEX>
{
  void reset(const Wrap<Type::COEDGE>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != SubType::COEDGE_REF, "Not expected coedge type");
    auto coedge_ref = static_cast<const Topo::CoEdgeRef*>(_from.get());
    for (int i = 0; i < 2; ++i)
    {
      auto v = static_cast<E<Type::VERTEX>*>(
        coedge_ref->loop()->get(Direction::Down, coedge_ref->ind_ + i));
      elems_.emplace_back(v);
    }
  }
};

namespace {

template <Type FromT>
struct IterToLoop : public BodyIteratorBase<Type::LOOP>
{
  void reset(const Wrap<FromT>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != TopoSubtype<FromT>::Value, "Wrong type");
    struct LoopAdder : public IterElement
    {
      std::vector<Wrap<Type::LOOP>>& elems_;
      LoopAdder(std::vector<Wrap<Type::LOOP>>& _elems) : elems_(_elems) {}
      virtual bool process(IBase* _parent, size_t _i)
      {
        if (_parent->type() < Type::LOOP)
          _parent = _parent->get(Direction::Up, _i);
        if (added_.insert(_parent).second)
        {
          Wrap<Type::LOOP> loop;
          auto loop_ref = loop.make<LoopRef>();
          loop_ref->set_loop(_parent);
          elems_.emplace_back(loop);
        }
        return true;
      };
      virtual bool add(IBase* _loop)
      {
        if (added_.insert(_loop).second)
          elems_.emplace_back(static_cast<EE<Type::LOOP>*>(_loop));
        return true;
      };
      std::set<IBase*> added_;
    };
    LoopAdder loop_adder(elems_);
    const Direction DIR = FromT > Type::LOOP ? Direction::Down : Direction::Up;
    topo_iterate<DIR, Type::LOOP>(
      static_cast<IBase*>(_from.get()), loop_adder);
  }
};

} // namespace

template <>
struct Iterator<Type::BODY, Type::LOOP>::Impl : public IterToLoop<Type::BODY> {};

template <>
struct Iterator<Type::FACE, Type::LOOP>::Impl : public IterToLoop<Type::FACE> {};

template <>
struct Iterator<Type::VERTEX, Type::LOOP>::Impl : public IterToLoop<Type::VERTEX> {};

template <>
struct Iterator<Type::LOOP, Type::VERTEX>::Impl : BodyIteratorBase<Type::VERTEX>
{
  void reset(const Wrap<Type::LOOP>& _from)
  {
    clear();
    IBase * ptr_base = nullptr;
    if (_from->sub_type() == SubType::LOOP)
      ptr_base = _from.get();
    else if (_from->sub_type() == SubType::LOOP_REF)
    {
      LoopRef* lr = static_cast<LoopRef*>(_from.get());
      ptr_base = lr->loop();
    }
    else
      THROW("Wrong type");
    AddIterElement<Type::VERTEX> vert_adder(elems_);
    topo_iterate<Direction::Down, Type::VERTEX>(ptr_base, vert_adder);
  };
};

template <>
struct Iterator<Type::LOOP, Type::FACE>::Impl : public BodyIteratorBase<Type::FACE>
{
  void reset(const Wrap<Type::LOOP>& _from)
  {
    clear();
    IBase * ptr_base = nullptr;
    if (_from->sub_type() == SubType::LOOP)
      ptr_base = _from.get();
    else if (_from->sub_type() == SubType::LOOP_REF)
    {
      LoopRef* lr = static_cast<LoopRef*>(_from.get());
      ptr_base = lr->loop();
    }
    else
      THROW("Wrong type");
    elems_.push_back(static_cast<E<Type::FACE>*>(ptr_base));
  }
};


template Iterator<Type::BODY, Type::FACE>;
template Iterator<Type::BODY, Type::EDGE>;
template Iterator<Type::BODY, Type::VERTEX>;
template Iterator<Type::FACE, Type::VERTEX>;
template Iterator<Type::FACE, Type::EDGE>;
template Iterator<Type::EDGE, Type::VERTEX>;
template Iterator<Type::VERTEX, Type::EDGE>;
template Iterator<Type::VERTEX, Type::FACE>;
template Iterator<Type::EDGE, Type::COEDGE>;
template Iterator<Type::COEDGE, Type::EDGE>;
template Iterator<Type::EDGE, Type::FACE>;
template Iterator<Type::COEDGE, Type::FACE>;
template Iterator<Type::BODY, Type::COEDGE>;
template Iterator<Type::FACE, Type::COEDGE>;
template Iterator<Type::LOOP, Type::COEDGE>;
template Iterator<Type::COEDGE, Type::VERTEX>;
template Iterator<Type::BODY, Type::LOOP>;
template Iterator<Type::FACE, Type::LOOP>;
template Iterator<Type::LOOP, Type::VERTEX>;
template Iterator<Type::LOOP, Type::FACE>;
template Iterator<Type::VERTEX, Type::LOOP>;


}//namespace Topo

