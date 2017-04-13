
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
    if (_from->sub_type() != TopoSubtype<FromT>::Value)
      throw;

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
    if (_from->sub_type() == SubType::EDGE)
    {
      THROW("NOT_IMPLEMENTED");
    }
    else if (_from->sub_type() == SubType::EDGE_REF)
    {
      auto ed_ref = static_cast<const EdgeRef*>(_from.get());
      for (size_t i = 0; i < std::size(ed_ref->verts_); ++i)
        elems_.push_back(ed_ref->verts_[i]);
    }
    else
      THROW("UNEXPECTED_EDGE_TYPE");
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
      if (face->type() == Type::FACE)
      {
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
  }
};

template <>
struct Iterator<Type::EDGE, Type::COEDGE>::Impl : public BodyIteratorBase<Type::COEDGE>
{
  void reset(const Wrap<Type::EDGE>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != SubType::EDGE_REF, "Not expected edge type");
    auto edge_ref = static_cast<const Topo::EdgeRef*>(_from.get());

    std::vector<Wrap<Type::FACE>> faces[2], common_faces;
    for (int i = 0; i < 2; ++i)
    {
      auto& vert = edge_ref->verts_[i];
      for (size_t j = 0; j < vert->size(Topo::Direction::Up); ++j)
      {
        auto ptr_face = vert->get(Topo::Direction::Up, j);
        THROW_IF(ptr_face->type() != Type::FACE, "Unexpected type");
        faces[i].emplace_back(static_cast<E<Type::FACE>*>(ptr_face));
      }
      std::sort(faces[i].begin(), faces[i].end());
      faces[i].erase(std::unique(faces[i].begin(), faces[i].end()), faces[i].end());
    }
    std::set_intersection(
      faces[0].begin(), faces[0].end(),
      faces[1].begin(), faces[1].end(),
      std::back_inserter(common_faces));
    for (auto& face : common_faces)
    {
      auto vert_ptr = edge_ref->verts_[0].get();
      for (auto pos = face->find_child(vert_ptr); pos != SIZE_MAX; pos = face->find_child(vert_ptr, pos))
      {
        size_t pos_near[2] = { pos , pos + 1 };
        if (pos_near[0] > 0) --pos_near[0];
        else pos_near[0] = face->size(Direction::Down) - 1;
        if (pos_near[1] >= face->size(Direction::Down)) pos_near[1] = 0;

        for (int i = 0; i < 2; ++i)
        {
          auto pos_oth = pos_near[i];
          auto vert_oth = face->get(Direction::Down, pos_oth);
          if (vert_oth->type() != Type::VERTEX)
            continue;
          if (vert_oth != edge_ref->verts_[1].get())
            continue;

          Wrap<Type::COEDGE> coedge;
          auto cedge_data = coedge.make<CoEdgeRef>();
          cedge_data->set_loop(face.get());
          cedge_data->ind_ = i > 0 ? pos : pos_near[0];
          elems_.emplace_back(coedge);
        }
      }
    }
  }
};

template <>
struct Iterator<Type::EDGE, Type::FACE>::Impl : public BodyIteratorBase<Type::FACE>
{
  void reset(const Wrap<Type::EDGE>& _from)
  {
    clear();
    THROW_IF(_from->sub_type() != SubType::EDGE_REF, "Not expected edge type");
    auto edge_ref = static_cast<const Topo::EdgeRef*>(_from.get());

    std::vector<Wrap<Type::FACE>> faces[2], common_faces;

    for (int i = 0; i < 2; ++i)
    {
      get_elements(edge_ref->verts_[i], faces[i]);
      std::sort(faces[i].begin(), faces[i].end());
    }
    std::set_intersection(
      faces[0].begin(), faces[0].end(),
      faces[1].begin(), faces[1].end(),
      std::back_inserter(common_faces));
    for (auto& face : common_faces)
    {
      auto vert_ptr = edge_ref->verts_[0].get();
      for (auto pos = face->find_child(vert_ptr); pos != SIZE_MAX; pos = face->find_child(vert_ptr, pos))
      {
        size_t pos_near[2] = { pos , pos + 1 };
        if (pos_near[0] > 0) --pos_near[0];
        else pos_near[0] = face->size(Direction::Down) - 1;
        if (pos_near[1] >= face->size(Direction::Down)) pos_near[1] = 0;

        for (int i = 0; i < 2; ++i)
        {
          auto pos_oth = pos_near[i];
          auto vert_oth = face->get(Direction::Down, pos_oth);
          if (vert_oth->type() != Type::VERTEX)
            continue;
          if (vert_oth != edge_ref->verts_[1].get())
            continue;
          elems_.emplace_back(face);
          break;
        }
      }
    }
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

template <>
struct Iterator<Type::FACE, Type::COEDGE>::Impl : public BodyIteratorBase<Type::COEDGE>
{
  void reset(const Wrap<Type::FACE>& _from)
  {
    clear();
    auto nverts = _from->size(Direction::Down);
    if (nverts < 2)
      return;
    for (auto i = 0; i < nverts; ++i)
    {
      Wrap<Type::COEDGE> co_edge;
      auto co_edref = co_edge.make<CoEdgeRef>();
      co_edref->set_loop(_from.get());
      co_edref->ind_ = i;
      elems_.emplace_back(co_edref);
    }
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
template Iterator<Type::FACE, Type::COEDGE>;
template Iterator<Type::COEDGE, Type::VERTEX>;

}//namespace Topo

