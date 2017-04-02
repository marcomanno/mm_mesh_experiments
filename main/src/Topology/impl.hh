#pragma once

#include "Topology.hh"

#include <vector>

namespace Topo {

template <Type typeT> struct Base : public E<typeT>
{
  virtual size_t size(Direction _dir) const
  {
    return _dir == Direction::Up ? up_elems_.size() : 0;
  }

  virtual IBase* get(Direction _dir, size_t _i) const
  {
    if (_dir == Direction::Up && !up_elems_.empty())
      return up_elems_[_i % up_elems_.size()];
    else
      return nullptr;
  }

  virtual bool reversed(Direction /*_dir*/, size_t /*_i*/) const
  {
    return false;
  }

  virtual bool replace(IBase* _new_elem)
  {
    auto up_elems = up_elems_;
    for (const auto& prnt : up_elems)
      prnt->replace_child(this, _new_elem);
    return true;
  }

  virtual bool remove()
  {
    auto up_elems = up_elems_;
    for (const auto& prnt : up_elems)
      prnt->remove_child(this);
    return true;
  }

  virtual size_t find_parent(const IBase* _prnt) const
  {
    auto it = std::find(up_elems_.begin(), up_elems_.end(), _prnt);
    if (it == up_elems_.end())
      return SIZE_MAX;
    return it - up_elems_.begin();
  }

protected:
  virtual bool remove_parent(IBase* _prnt)
  {
    auto it = std::find(up_elems_.begin(), up_elems_.end(), _prnt);
    if (it == up_elems_.end())
      return false;
    up_elems_.erase(it);
    return true;
  }

  bool add_parent(IBase* _prnt) { up_elems_.push_back(_prnt); return true; }

  std::vector<IBase*> up_elems_;
};

template <Type typeT> struct UpEntity : public Base<typeT>
{
  ~UpEntity()
  {
    for (auto el : low_elems_)
      el->release_ref();
  }

  virtual size_t size(Direction _dir) const
  {
    return _dir == Direction::Up ? up_elems_.size() : low_elems_.size();
  }

  virtual IBase* get(Direction _dir, size_t _i) const
  {
    if (_dir == Direction::Up)
      return up_elems_.empty() ? nullptr : up_elems_[_i % up_elems_.size()];
    else
      return low_elems_.empty() ? nullptr : low_elems_[_i % low_elems_.size()];
  }

  virtual bool insert_child(IBase* _el, size_t _pos = SIZE_MAX)
  {
    if (_el == nullptr)
      return false;
    auto it = (_pos >= low_elems_.size()) ? low_elems_.end() : low_elems_.begin() + _pos;
    low_elems_.insert(it, _el);
    _el->add_ref();
    _el->add_parent(this);
    return true;
  }

  virtual bool remove_child(size_t _pos)
  {
    if (_pos >= low_elems_.size())
      return false;
    auto obj = low_elems_[_pos];
    low_elems_.erase(low_elems_.begin() + _pos);
    obj->remove_parent(this);
    obj->release_ref();
    return true;
  }

  virtual bool remove_child(IBase* _el)
  {
    return remove_child(find_child(_el));
  }

  virtual bool replace_child(size_t _pos, IBase* _new_obj)
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

  virtual bool replace_child(IBase* _el, IBase* _new_el)
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
  virtual size_t find_child(const IBase* _el, size_t _end = SIZE_MAX) const
  {
    auto start_it = low_elems_.rbegin();
    if (_end < low_elems_.size())
      start_it += low_elems_.size() - _end;
    auto it = std::find(start_it, low_elems_.rend(), _el);
    if (it == low_elems_.rend())
      return SIZE_MAX;
    return low_elems_.rend() - it - 1;
  }

  virtual bool remove()
  {
    for (size_t i = size(Direction::Down); i-- > 0;)
      remove_child(i);
    return Base<typeT>::remove();
  }

protected:
  std::vector<IBase*> low_elems_;
};

template <Type typeT> struct EE;

template <> struct EE<Type::BODY> : public UpEntity<Type::BODY>
{
  virtual SubType sub_type() const { return SubType::BODY; }
  virtual bool insert_child(IBase* _el, size_t _pos = SIZE_MAX);
  virtual bool replace_child(size_t _pos, IBase* _new_obj);
  // search for an element in the range [0, _end[ in reverse order.
  virtual size_t find_child(const IBase* _el, size_t _end = SIZE_MAX) const;
  virtual void optimize();
  virtual bool remove();
  virtual bool EE<Type::BODY>::remove_children(
    std::vector<IBase*>& _faces_to_remove);

  bool ordered_children_ = true;
};

template <> struct EE<Type::FACE> : public UpEntity<Type::FACE>
{
  virtual SubType sub_type() const { return SubType::FACE; }
  virtual bool reverse();
  virtual Geo::Point internal_point() const;
  virtual Geo::Range<3> box() const;
  bool check();
};

template <> struct EE<Type::LOOP> : public UpEntity<Type::LOOP>
{
  virtual SubType sub_type() const { return SubType::LOOP; }
};

template <> struct EE<Type::EDGE> : public UpEntity<Type::EDGE>
{
  virtual SubType sub_type() const { return SubType::EDGE; }
  virtual bool geom(Geo::Segment&) const;
  virtual bool set_geom(const Geo::Segment&);
  virtual double tolerance() const { return tol_; }
  virtual bool set_tolerance(const double _tol) { tol_ = _tol; return true; }
  double tol_ = 0;
};

template <> struct EE<Type::VERTEX> : public Base<Type::VERTEX>
{
  virtual bool geom(Geo::Point& _pt) const { _pt = pt_; return true; }
  virtual bool set_geom(const Geo::Point& _pt) { pt_ = _pt; return true; }
  virtual double tolerance() const;
  virtual bool set_tolerance(const double _tol) { tol_ = _tol; return true;  }
  virtual SubType sub_type() const { return SubType::VERTEX; }
  virtual Geo::Point internal_point() const { return pt_; }
  virtual Geo::Range<3> box() const;
private:
  Geo::Point pt_;
  double tol_ = 0;
};

struct EdgeRef : public E<Type::EDGE>
{
  Wrap<Type::VERTEX> verts_[2];  // Edge vertices.

  virtual SubType sub_type() const { return SubType::EDGE_REF; }
  virtual bool geom(Geo::Segment& _seg) const;
  virtual bool set_geom(const Geo::Segment&) { return false; }
  virtual double tolerance() const;
  virtual bool set_tolerance(const double) { return false; }
  virtual Geo::Point internal_point() const;
  virtual Geo::Range<3> box() const;

  virtual bool operator<(const Object& _oth) const;
  virtual bool operator==(const Object& _oth) const;
  void finalise();
  virtual bool operator!=(const EdgeRef& _oth) const { return !(*this == _oth); }
};

struct CoEdgeRef : public E<Type::COEDGE>
{
  Wrap<Type::FACE> face_;  // Face.
  size_t ind_ = 0;
  virtual SubType sub_type() const { return SubType::COEDGE_REF; }
  virtual bool geom(Geo::Segment&) const;
  virtual bool set_geom(const Geo::Segment&) { return false; }
  virtual double tolerance() const;
  virtual bool set_tolerance(const double) { return false; }
  virtual bool operator<(const Object& _oth) const;
  virtual bool operator==(const Object& _oth) const;
  virtual bool operator!=(const EdgeRef& _oth) const { return !(*this == _oth); }
};

struct LoopRef : public E<Type::LOOP>
{
  Wrap<Type::FACE> face_;  // Face.
  virtual SubType sub_type() const { return SubType::LOOP_REF; }
};

}//namespace Topo
