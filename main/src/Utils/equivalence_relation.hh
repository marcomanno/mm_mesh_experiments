#pragma once

#include <map>
#include <vector>

namespace Utils
{

template <class ObjectT>
class EquivalenceRelations
{
  using Relations = std::map<ObjectT, ObjectT>;
  struct EquivalentIteration
  {
    virtual bool process(const typename Relations::iterator&) = 0;
  };

  typename Relations::iterator loop_over_iteration(
    typename Relations::iterator _start)
  {
    auto it_it = _start;
    for (; it_it->second != _start->first; it_it = relations_.find(it_it->second));
    return it_it;
  }

  typename Relations::iterator loop_over_iteration(
    typename Relations::iterator _start,
    EquivalentIteration& _call_back)
  {
    auto it_it = _start;
    auto res = _start;
    do {
      if (!_call_back.process(it_it))
        return _start; // error
      res = it_it;
      it_it = relations_.find(it_it->second);
    } while (it_it != _start);
    return res;
  }

public:
  bool add_relation(const ObjectT& _a, const ObjectT& _b)
  {
    if (_a == _b)
      return false;
    auto it_a = relations_.find(_a);
    auto it_b = relations_.find(_b);
    if (it_a == relations_.end() && it_b == relations_.end())
    {
      relations_[_a] = _b;
      relations_[_b] = _a;
      return true;
    }
    if (it_a != relations_.end() && it_b != relations_.end())
    {
      struct CheckNot : public EquivalentIteration
      {
        CheckNot(const ObjectT& _a) : obj_(_a) {}
        bool process(const typename Relations::iterator& _it) override
        {
          return _it->second != obj_;
        }
        const ObjectT& obj_;
      };
      CheckNot cn(_b);
      auto it_to_a = loop_over_iteration(it_a, cn);
      if (it_to_a == it_a)
        return false;
      auto it_to_b = loop_over_iteration(it_b);
      it_to_a->second = _b;
      it_to_b->second = _a;
      return true;
    }
    if (it_b == relations_.end())
    {
      relations_[_b] = _a;
      auto it_to_a = loop_over_iteration(it_a);
      it_to_a->second = _b;
    }
    else
    {
      relations_[_a] = _b;
      auto it_to_b = loop_over_iteration(it_b);
      it_to_b->second = _a;
    }
    return true;
  }

  std::vector<ObjectT> extract_equivalence_set()
  {
    if (relations_.empty())
      return std::vector<ObjectT>();
    struct MakeResult : public EquivalentIteration
    {
      bool process(const typename Relations::iterator& _it) override
      {
        res_.push_back(_it->first);
        return true;
      }
      std::vector<ObjectT> res_;
    };
    MakeResult res;
    loop_over_iteration(relations_.begin(), res);
    for (auto& obj : res.res_)
      relations_.erase(obj);
    return res.res_;
  }

  Relations relations_;
};

} // namespace Utils
