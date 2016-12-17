
#pragma once

#include <boost/intrusive/set.hpp>

namespace Base {

template <class Obj>
class LookupList
{
  struct ObjElement : public Obj,
    public boost::intrusive::set_base_hook
    <boost::intrusive::link_mode<boost::intrusive::auto_unlink>>
  {
    using Obj::Obj;
    ObjElement* links_[2] = {this, this};
  };

  template <class Element>  class iterator_base :
    public std::iterator<std::bidirectional_iterator_tag, Element>
  {
    pointer pos_;
  public:
    iterator_base(Element* v = nullptr) : pos_(v) {}
    reference operator* () const { return *pos_; }
    pointer   operator->() const { return pos_; }
    bool operator==(const iterator_base& rhs) const
    { return pos_ == rhs.pos_; }
    bool operator!=(const iterator_base& rhs) const
    { return pos_ != rhs.pos_; }
    iterator_base& operator++() { pos_ = pos_->links_[1]; return *this; }
    iterator_base& operator--() { pos_ = pos_->links_[0]; return *this; }
  };
public:
  typedef typename iterator_base<const ObjElement> iterator;
  typedef typename iterator_base<const ObjElement> const_iterator;
  ~LookupList() { clear(); }

  iterator begin()
  { 
    if (root_.links_[1] != nullptr)
      return root_.links_[1];
    else
      return &root_;
  }
  iterator end() { return &root_; }
  const_iterator cbegin() const { return root_.links_[1]; }
  const_iterator cend() const { return &root_; }
  bool empty() const { return root_.links_[1] == nullptr; }
  size_t size() const { return set_.size(); }
  void clear()
  {
    for (auto p = root_.links_[1]; p != &root_; )
    {
      auto q = p;
      p = p->links_[1];
      delete q;
    }
  }

  template< class... Args >
  iterator emplace(iterator pos, Args&&... args)
  {
    auto p = std::make_unique<ObjElement>(std::forward<Args>(args)...);
    if (lookup(*p) != end())
      return end(); // object is already present.
    auto obj_ptr = const_cast<ObjElement*>(&(*pos));
    p->links_[1] = obj_ptr;
    p->links_[0] = pos->links_[0];
    obj_ptr->links_[0] = p.get();
    p->links_[0]->links_[1] = p.get();
    set_.insert(*p);
    return p.release();
  }

  template<class... Args>
  iterator emplace_back(Args&&... args)
  {
    return emplace(end(), std::forward<Args>(args)...);
  }
  template< class... Args >
  iterator emplace_front(Args&&... args)
  {
    return emplace(begin(), std::forward<Args>(args)...);
  }
  iterator erase(iterator _pos)
  {
    _pos->links_[1]->links_[0] = _pos->links_[0];
    _pos->links_[0]->links_[1] = _pos->links_[1];
    auto next = _pos->links_[1];
    delete &(*_pos);
    return next;
  }

  iterator lookup(const Obj& _obj)
  {
    auto pos = set_.find(ObjElement(_obj));
    if (pos == set_.end())
      return end();
    return &(*pos);
  }

private:
  boost::intrusive::set<ObjElement, 
    boost::intrusive::constant_time_size<false>> set_;
  ObjElement root_;
};

}//namespace Base