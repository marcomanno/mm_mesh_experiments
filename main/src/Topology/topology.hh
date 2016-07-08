#pragma once

#include "subtype.hh"
#include "Utils/enum.hh"
#include "Geo/entity.hh"

#include <array>
#include <vector>

namespace Topo {

MAKE_ENUM(Type, VERTEX, EDGE, COEDGE, FACE, SHELL, BODY)

MAKE_ENUM(Direction, Up, Down );

struct Object
{
  template <Type typeT> friend class Wrap;

  void add_ref() { ++ref_; }
  void release_ref() { if (--ref_ == 0) delete this; }
  virtual Type type() const = 0;
  virtual SubType sub_type() const = 0;

  virtual bool operator<(const Object& _oth) const;
  virtual bool operator==(const Object& _oth) const;

protected:
  Object();
  virtual ~Object();

private:
  static void* operator new(std::size_t sz) { return ::operator new(sz); }
  static void* operator new[](std::size_t sz) { return ::operator new(sz); }

private:
  size_t ref_;
  size_t id_;
};

struct IBase : public Object
{
  template <Type typeT> friend struct UpEntity;

  virtual bool replace(IBase*) { return false; }
  virtual bool remove() { return false; }
  virtual bool reverse() { return false; }

  virtual size_t size(Direction) const { return SIZE_MAX; }

  virtual IBase* get(Direction, size_t) const { return nullptr; }

  virtual size_t find_parent(const IBase*) const { return SIZE_MAX; }
  virtual size_t find_child(const IBase*, size_t _end = SIZE_MAX) const { _end; return SIZE_MAX; }
  virtual bool insert_child(IBase*, size_t _pos = SIZE_MAX) { _pos; return false; }
  virtual bool remove_child(size_t) { return false; }
  virtual bool remove_child(IBase*) { return false; }
  virtual bool replace_child(IBase* /*_elem*/, IBase* /*_new_elem*/) { return false; }
protected:
  virtual bool remove_parent(IBase* /*_prnt*/) { return false; }
  virtual bool add_parent(IBase* /*_prnt*/) { return false; }
};

template <Type typeT> struct E;

template <> struct E<Type::VERTEX> : public IBase
{
  virtual bool geom(Geo::Point&) const = 0;
  virtual bool set_geom(const Geo::Point&) = 0;
  virtual double tolerance() const = 0;
  virtual bool set_tolerance(const double _tol) = 0;

  virtual Type type() const { return Type::VERTEX; };
};

template <> struct E<Type::EDGE> : public IBase
{
  virtual Type type() const { return Type::EDGE; };
  virtual bool geom(Geo::Segment&) const = 0;
  virtual bool set_geom(const Geo::Segment&) = 0;
  virtual double tolerance() const = 0;
  virtual bool set_tolerance(const double _tol) = 0;
};

template <> struct E<Type::FACE> : public IBase
{
  virtual Type type() const { return Type::FACE; };
};

template <> struct E<Type::BODY> : public IBase
{
  virtual Type type() const { return Type::BODY; };
};

template <> struct E<Type::COEDGE> : public IBase
{
  virtual Type type() const { return Type::COEDGE; };
  virtual bool geom(Geo::Segment&) const = 0;
  virtual bool set_geom(const Geo::Segment&) = 0;
  virtual double tolerance() const = 0;
  virtual bool set_tolerance(const double _tol) = 0;
};

template <Type typeT> class Wrap
{
public:
  Wrap() : ptr_(nullptr) { }

  Wrap(E<typeT>* _ptr) : ptr_(nullptr) { reset(_ptr); }

  template <class TopoTypeT> TopoTypeT* make()
  {
    auto ptr = new TopoTypeT;
    reset(ptr);
    return ptr;
  }

  Wrap(const Wrap<typeT>& _ew) : ptr_(_ew.ptr_)
  {
    if (ptr_ != nullptr)
      ptr_->add_ref();
  }

  ~Wrap()
  {
    if (ptr_)
      ptr_->release_ref();
  }

  Wrap& operator=(const Wrap& _oth)
  {
    reset(_oth.ptr_);
    return *this;
  }

  E<typeT>* operator->() { return get(); }

  const E<typeT>* operator->() const { return get(); }

  E<typeT>* get() { return ptr_; }

  const E<typeT>* get() const { return ptr_; }

  explicit operator bool() const { return ptr_ != nullptr; }

  void reset(E<typeT>* _ptr)
  {
    if (ptr_ != _ptr)
    {
      if (ptr_ != nullptr) ptr_->release_ref();
      ptr_ = _ptr;
      if (ptr_ != nullptr) ptr_->add_ref();
    }
  }

  bool operator<(const Wrap<typeT>& _oth) const 
  {
    if (ptr_ == _oth.ptr_)
      return false;
    if (ptr_ != nullptr && _oth.ptr_ != nullptr)
      return *ptr_ < *_oth.ptr_;
    return ptr_ == nullptr;
  }

  bool operator==(const Wrap<typeT>& _oth) const 
  {
    if (ptr_ == _oth.ptr_)
      return true;
    if (ptr_ != nullptr && _oth.ptr_ != nullptr)
      return *ptr_ == *_oth.ptr_;
    return false;
  }

  bool operator!=(const Wrap<typeT>& _oth) const { return !(*this == _oth); }

private:
  E<typeT>* ptr_;
};

typedef std::vector<Topo::Wrap<Topo::Type::VERTEX>> VertexChain;
typedef std::vector<VertexChain> VertexChains;

}//namespace Topo
