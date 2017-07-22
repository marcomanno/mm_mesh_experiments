#pragma once

#include "Topology.hh"

#include <vector>

namespace Topo {

template <Type typeT> struct Base : public E<typeT>
{
  virtual size_t size(Direction _dir) const;

  virtual IBase* get(Direction _dir, size_t _i) const;

  virtual bool reversed(Direction /*_dir*/, size_t /*_i*/) const;

  virtual bool replace(IBase* _new_elem);

  virtual bool remove();

  virtual size_t find_parent(const IBase* _prnt) const;

protected:
  virtual bool remove_parent(IBase* _prnt);
  bool add_parent(IBase* _prnt);

  std::vector<IBase*> up_elems_;
};

template <Type typeT> struct UpEntity : public Base<typeT>
{
  ~UpEntity();

  virtual size_t size(Direction _dir) const;

  virtual IBase* get(Direction _dir, size_t _i) const;

  virtual bool insert_child(IBase* _el, size_t _pos = SIZE_MAX);

  virtual bool remove_child(size_t _pos);

  virtual bool remove_child(IBase* _el);

  virtual bool replace_child(size_t _pos, IBase* _new_obj);

  virtual bool replace_child(IBase* _el, IBase* _new_el);

  // search for an element in the range [0, _end[ in reverse order.
  virtual size_t find_child(const IBase* _el, size_t _end = SIZE_MAX) const;

  virtual bool remove();

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

#if 0 // No edge yet
template <> struct EE<Type::EDGE> : public UpEntity<Type::EDGE>
{
  virtual SubType sub_type() const { return SubType::EDGE; }
  virtual bool geom(Geo::Segment&) const { return false;  }
  virtual bool set_geom(const Geo::Segment&) { return false; };
  virtual double tolerance() const { return tol_; }
  virtual bool set_tolerance(const double _tol) { tol_ = _tol; return true; }
  double tol_ = 0;
};
#endif

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
  size_t ind_ = 0;

  IBase* loop() const { return loop_.get(); }
  void set_loop(IBase* _loop);
  virtual SubType sub_type() const { return SubType::COEDGE_REF; }
  virtual bool geom(Geo::Segment&) const;
  virtual bool set_geom(const Geo::Segment&) { return false; }
  virtual double tolerance() const;
  virtual bool set_tolerance(const double) { return false; }
  virtual bool operator<(const Object& _oth) const;
  virtual bool operator==(const Object& _oth) const;
  virtual bool operator!=(const EdgeRef& _oth) const { return !(*this == _oth); }
private:
  WrapIbase<IBase> loop_;  // Face or Loop.
};

struct LoopRef : public E<Type::LOOP>
{
  IBase* loop() const;
  void set_loop(IBase* _loop);
  virtual SubType sub_type() const { return SubType::LOOP_REF; }

  // From EE<FACE>
  virtual bool reverse() override;
  virtual Geo::Point internal_point() const override;
  virtual Geo::Range<3> box() const override;
  //bool check() { return loop_->check(); }

  // From UpEntity
  virtual size_t size(Direction _dir) const override { return loop_->size(_dir); }
  virtual IBase* get(Direction _dir, size_t _i) const override { return loop_->get(_dir, _i); }
  virtual bool insert_child(IBase* _el, size_t _pos = SIZE_MAX) override 
  { return loop_->insert_child(_el, _pos); }
  virtual bool remove_child(size_t _pos) override
  { return loop_->remove_child(_pos); }

  virtual bool remove_child(IBase* _el) override
  { return loop_->remove_child(_el); }
  //virtual bool replace_child(size_t _pos, IBase* _new_obj) override
  //{ return loop_->replace_child(_pos, _new_obj); }
  virtual bool replace_child(IBase* _el, IBase* _new_el) override
  { return loop_->replace_child(_el, _new_el); }

  // search for an element in the range [0, _end[ in reverse order.
  virtual size_t find_child(const IBase* _el, size_t _end = SIZE_MAX) const override
  { return loop_->find_child(_el, _end); }
  virtual bool remove() override
  { return loop_->remove(); }


  // From Base
  //virtual bool reversed(Direction _dir, size_t _i) const override
  //{ return loop_->reversed(_dir, _i); }
  virtual bool replace(IBase* _new_elem) override
  { return loop_->replace(_new_elem); }
  virtual size_t find_parent(const IBase* _prnt) const override
  { return loop_->find_parent(_prnt); }

  // From Object
  Identifier id() const override { return loop_->id(); }

  bool operator==(const Object& _oth) const
  {
    if (sub_type() != _oth.sub_type())
      return false;
    return *loop() == *static_cast<const LoopRef&>(_oth).loop();
  }

private:
  WrapIbase<IBase> loop_;
};

}//namespace Topo
