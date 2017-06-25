#pragma once

#include "subtype.hh"
#include "Geo/entity.hh"
#include "Geo/range.hh"


#include <array>
#include <vector>

namespace Topo {

MAKE_ENUM(Type, VERTEX, EDGE, COEDGE, LOOP, FACE, SHELL, BODY)

MAKE_ENUM(Direction, Up, Down )

typedef unsigned __int64 Identifier;

struct Object
{
  template <Type typeT> friend class Wrap;

  void add_ref() const
  {
    ++ref_;
  }
  void release_ref() const
  {
    auto refs = --ref_;
    if (refs == 0)
      delete this;
  }
  virtual Type type() const = 0;
  virtual SubType sub_type() const = 0;

  virtual bool operator<(const Object& _oth) const;
  virtual bool operator==(const Object& _oth) const;

  virtual Identifier id() const { return id_; }

protected:
  Object();
  virtual ~Object();

private:
  static void* operator new(std::size_t sz) { return ::operator new(sz); }
  static void* operator new[](std::size_t sz) { return ::operator new(sz); }

private:
  mutable size_t ref_;
  Identifier id_;
};

class WrapObject
{
  Object* ptr_ = nullptr;

public:
  WrapObject() { }
  WrapObject(Object* _ptr) { reset(_ptr); }
  WrapObject(const WrapObject& _oth)
  {
    reset(_oth.ptr_);
  }
  ~WrapObject()
  {
    if (ptr_)
      ptr_->release_ref();
  }

  WrapObject& operator=(const WrapObject& _oth)
  {
    reset(_oth.ptr_);
    return *this;
  }

  Object* operator->() { return get(); }

  const Object* operator->() const { return get(); }

  Object* get() { return ptr_; }

  const Object* get() const { return ptr_; }

  explicit operator bool() const { return ptr_ != nullptr; }

  void reset(Object* _ptr)
  {
    if (ptr_ != _ptr)
    {
      if (ptr_ != nullptr) ptr_->release_ref();
      ptr_ = _ptr;
      if (ptr_ != nullptr) ptr_->add_ref();
    }
  }

  bool operator<(const WrapObject& _oth) const
  {
    if (ptr_ == _oth.ptr_)
      return false;
    if (ptr_ != nullptr && _oth.ptr_ != nullptr)
      return *ptr_ < *_oth.ptr_;
    return ptr_ == nullptr;
  }

  bool operator==(const WrapObject& _oth) const
  {
    if (ptr_ == _oth.ptr_)
      return true;
    if (ptr_ != nullptr && _oth.ptr_ != nullptr)
      return *ptr_ == *_oth.ptr_;
    return false;
  }

  bool operator!=(const WrapObject& _oth) const { return !(*this == _oth); }

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
  virtual Geo::Point internal_point() const
  { return Geo::uniform_vector<3>(std::numeric_limits<double>::max()); }
  virtual Geo::Range<3> box() const { return Geo::Range<3>(); }
  virtual double tolerance() const { return 0; }
protected:
  virtual bool remove_parent(IBase* /*_prnt*/) { return false; }
  virtual bool add_parent(IBase* /*_prnt*/) { return false; }
};

template <Type typeT> struct EBase : public IBase
{
  virtual Type type() const { return typeT; }
};

template <Type typeT> struct E : public EBase<typeT> {};

template <> struct E<Type::VERTEX> : public EBase<Type::VERTEX>
{
  virtual bool geom(Geo::Point&) const = 0;
  virtual bool set_geom(const Geo::Point&) = 0;
  virtual double tolerance() const = 0;
  virtual bool set_tolerance(const double _tol) = 0;
};

template <> struct E<Type::EDGE> : public EBase<Type::EDGE>
{
  virtual bool geom(Geo::Segment&) const = 0;
  virtual bool set_geom(const Geo::Segment&) = 0;
  virtual double tolerance() const = 0;
  virtual bool set_tolerance(const double _tol) = 0;
};

template <> struct E<Type::BODY> : public EBase<Type::BODY>
{
  virtual void optimize() {}
  virtual bool remove_children(
    std::vector<IBase*>&)
  {
    return false;
  }
};

template <> struct E<Type::COEDGE> : public EBase<Type::COEDGE>
{
  virtual bool geom(Geo::Segment&) const = 0;
  virtual bool set_geom(const Geo::Segment&) = 0;
  virtual double tolerance() const = 0;
  virtual bool set_tolerance(const double _tol) = 0;
};

template <typename IBaseT> class WrapIbase
{
public:
  WrapIbase() : ptr_(nullptr) { }

  WrapIbase(IBaseT* _ptr) : ptr_(nullptr) { reset(_ptr); }

  WrapIbase(const WrapIbase<IBaseT>& _ew) : ptr_(_ew.ptr_)
  {
    if (ptr_ != nullptr)
      ptr_->add_ref();
  }

  ~WrapIbase()
  {
    if (ptr_)
      ptr_->release_ref();
  }

  WrapIbase& operator=(const WrapIbase<IBaseT>& _oth)
  {
    reset(_oth.ptr_);
    return *this;
  }

  IBaseT* operator->() { return get(); }

  const IBaseT* operator->() const { return get(); }

  IBaseT* get() const { return ptr_; }

  explicit operator bool() const { return ptr_ != nullptr; }

  void reset(IBaseT* _ptr)
  {
    if (ptr_ != _ptr)
    {
      if (ptr_ != nullptr) ptr_->release_ref();
      ptr_ = _ptr;
      if (ptr_ != nullptr) ptr_->add_ref();
    }
  }

  bool operator<(const WrapIbase<IBaseT>& _oth) const
  {
    if (ptr_ == _oth.ptr_)
      return false;
    if (ptr_ != nullptr && _oth.ptr_ != nullptr)
      return *ptr_ < *_oth.ptr_;
    return ptr_ == nullptr;
  }

  bool operator==(const WrapIbase<IBaseT>& _oth) const
  {
    if (ptr_ == _oth.ptr_)
      return true;
    if (ptr_ != nullptr && _oth.ptr_ != nullptr)
      return *ptr_ == *_oth.ptr_;
    return false;
  }

  bool operator!=(const WrapIbase<IBaseT>& _oth) const { return !(*this == _oth); }

private:
  IBaseT* ptr_;
};

template <Type typeT> class Wrap : public WrapIbase<E<typeT>>
{
public:
  using WrapIbase<E<typeT>>::WrapIbase;

  template <class TopoTypeT> TopoTypeT* make()
  {
    auto ptr = new TopoTypeT;
    reset(ptr);
    return ptr;
  }
};

typedef std::vector<Topo::Wrap<Topo::Type::VERTEX>> VertexChain;
typedef std::vector<VertexChain> VertexChains;

}//namespace Topo
