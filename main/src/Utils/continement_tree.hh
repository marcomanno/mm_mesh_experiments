#pragma once

#include <functional>
#include <memory>
#include <vector>

namespace Utils {

// Implement a tree where any alement can have children and relatives.
// It is useful to represent nested data, for example not intersecting loops.
template <class ElemT>
struct ContainementTree
{
  struct Element
  {
    friend struct ContainementTree;
    Element(const ElemT& _elem) : elem_(_elem) { }
    const Element* next() const { return next_.get(); }
    const Element* child() const { return child_.get(); }
    const ElemT& data() const { return elem_; }
    operator const ElemT&() const { return elem_; }
  private:
    ElemT elem_;
    std::unique_ptr<Element> next_;
    std::unique_ptr<Element> child_;
  };
  using CompareFunc = std::function<int(const ElemT&, const ElemT&)>;

  ContainementTree(CompareFunc _cf) : cmp_func_(_cf) {}

  const Element* root() const { return root_.get(); }

  void add(const ElemT& _elem)
  {
    auto new_node = new Element(_elem);
    std::unique_ptr<Element>* ins_el = &root_;
    while (*ins_el)
    {
      auto res = cmp_func_(_elem, (*ins_el)->elem_);
      if (res == 0)
        ins_el = &(*ins_el)->next_;
      else if (res < 0)
        ins_el = &(*ins_el)->child_;
      else // if (res > 0)
      {
        auto move_to = &new_node->child_;
        bool first_iter = true;
        while (*ins_el)
        {
          if (first_iter)
            first_iter = false;
          else
            res = cmp_func_(_elem, (*ins_el)->elem_);
          if (res <= 0)
            ins_el = &(*ins_el)->next_;
          else
          {
            std::swap(*ins_el, *move_to);
            std::swap(*ins_el, (*move_to)->next_);
            move_to = &(*move_to)->next_;
          }
        }
      }
    }
    (*ins_el).reset(new_node);
  }

private:
  std::unique_ptr<Element> root_;
  CompareFunc cmp_func_;
};

}