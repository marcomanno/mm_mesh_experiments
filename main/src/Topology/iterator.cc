
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

template <Type typeT> struct BodyIteratorBase
{
  size_t size() const { return elems_.size(); }
  Wrap<typeT> get(size_t _i) const { return elems_[_i]; }
  Wrap<typeT>* begin() { return elems_.data(); }
  Wrap<typeT>* end() { return elems_.data() + elems_.size(); }
  void clear() { elems_.clear();  }
  std::vector<Wrap<typeT>> elems_;
};

}//namespace

template <>
struct Iterator<Type::BODY, Type::FACE>::Impl : public BodyIteratorBase<Type::FACE>
{
  void reset(const Wrap<Type::BODY>& _from)
  {
    clear();
    if (_from->sub_type() != SubType::BODY)
      throw;
    auto body = static_cast<const EE<Type::BODY>*>(_from.get());
    
    auto face_nmbr = body->size(Direction::Down);
    elems_.reserve(face_nmbr);
    for (size_t i = 0; i < face_nmbr; ++i)
    {
      auto child = body->get(Direction::Down, i);
      if (child->type() == Type::FACE)
        elems_.emplace_back(static_cast<EE<Type::FACE>*>(child));
    }
  }
};

template <> 
struct Iterator<Type::BODY, Type::EDGE>::Impl : public BodyIteratorBase<Type::EDGE>
{
  void reset(const Wrap<Type::BODY>& _from)
  {
    clear();
    if (_from->sub_type() != SubType::BODY)
      throw;
    auto body = static_cast<const EE<Type::BODY>*>(_from.get());
    auto face_nmbr = body->size(Direction::Down);
    typedef std::array<Object*, 2> Key;
    typedef std::map<Key, Wrap<Type::EDGE>> EdgeMap;
    EdgeMap edges;
    for (size_t i = 0; i < face_nmbr; ++i)
    {
      auto child = body->get(Direction::Down, i);
      if (child->type() != Type::FACE)
        continue;
      auto face = static_cast<EE<Type::FACE>*>(child);
      auto edge_nmbr = face->size(Direction::Down);
      Key verts;
      verts[0] = face->get(Direction::Down, edge_nmbr - 1);
      for (size_t j = 0; j < edge_nmbr; ++j)
      {
        Wrap<Type::EDGE> edg_wrp;
        auto edg = edg_wrp.make<EdgeRef>();
        verts[1] = face->get(Direction::Down, j);

        for (size_t k = 0; k < 2; ++k)
        {
          THROW_IF(verts[k]->type() != Type::VERTEX, "Unexpected type");
          edg->verts_[k] = static_cast<E<Type::VERTEX>*>(verts[k]);
        }
        edg->finalise();

        Key key(verts);
        if (key[0] > key[1])
          std::swap(key[0], key[1]);
        auto it = edges.lower_bound(key);
        if (it == edges.end() || it->first != key)
          edges.insert(it, EdgeMap::value_type(key, edg_wrp));
        verts[0] = verts[1];
      }
    }
    for (auto ed : edges)
      elems_.push_back(ed.second);
  }
};

template <> 
struct Iterator<Type::BODY, Type::VERTEX>::Impl : public BodyIteratorBase<Type::VERTEX>
{
  void reset (const Wrap<Type::BODY>& _from)
  {
    clear();
    if (_from->sub_type() != SubType::BODY)
      throw;
    auto body = static_cast<const EE<Type::BODY>*>(_from.get());
    auto face_nmbr = body->size(Direction::Down);
    std::set<EE<Type::VERTEX>*> vertices;
    for (size_t i = 0; i < face_nmbr; ++i)
    {
      auto child = body->get(Direction::Down, i);
      if (child->type() != Type::FACE)
        continue;
      auto face = static_cast<EE<Type::FACE>*>(child);
      auto vert_nmbr = face->size(Direction::Down);
      for (size_t j = 0; j < vert_nmbr; ++j)
      {
        auto elem = face->get(Direction::Down, j);
        if (elem->type() != Type::VERTEX)
          continue;
        auto vert = static_cast<EE<Type::VERTEX>*>(elem);
        if (vertices.insert(vert).second)
          elems_.push_back(vert);
      }
    }
  }
};

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
struct Iterator<Type::FACE, Type::VERTEX>::Impl : public BodyIteratorBase<Type::VERTEX>
{
  void reset(const Wrap<Type::FACE>& _from)
  {
    clear();
    for (size_t i = 0; i < _from->size(Direction::Down); ++i)
    {
      auto vert = _from->get(Direction::Down, i);
      if (vert->type() == Type::VERTEX)
        elems_.emplace_back(static_cast<E<Type::VERTEX>*>(vert));
    }
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
          cedge_data->face_ = face;
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
      auto& vert = edge_ref->verts_[i];
      for (size_t j = 0; j < vert->size(Topo::Direction::Up); ++j)
      {
        auto ptr_face = vert->get(Topo::Direction::Up, j);
        THROW_IF(ptr_face->type() != Type::FACE, "Unexpected type");
        faces[i].emplace_back(static_cast<E<Type::FACE>*>(ptr_face));
      }
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
    elems_.emplace_back(coedge_ref->face_);
  }
};

template <>
struct Iterator<Type::FACE, Type::EDGE>::Impl : public BodyIteratorBase<Type::EDGE>
{
  void reset(const Wrap<Type::FACE>& _from)
  {
    clear();
    auto nverts = _from->size(Direction::Down);
    if (nverts < 2)
      return;
    auto prev = _from->get(Direction::Down, nverts - 1);
    for (auto i = 0; i < nverts; ++i)
    {
      Wrap<Type::EDGE> edge;
      auto edref = edge.make<EdgeRef>();
      edref->verts_[0].reset(static_cast<E<Type::VERTEX>*>(prev));
      prev = _from->get(Direction::Down, i);
      edref->verts_[1].reset(static_cast<E<Type::VERTEX>*>(prev));
      edref->finalise();
      elems_.emplace_back(edge);
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
template Iterator<Type::EDGE, Type::COEDGE>;
template Iterator<Type::EDGE, Type::FACE>;
template Iterator<Type::COEDGE, Type::FACE>;

}//namespace Topo

