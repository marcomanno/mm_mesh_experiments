#include "graph.hh"

#include <array>
#include <vector>
#include  <set>

#include <boost/bimap.hpp>

namespace Utils {

class GraphBase::Impl
{
  typedef std::set<std::array<int, 2>> Connections;
  Connections links_;
  std::vector<std::vector<const void*>> chain_list_;
  bool computed_ = true;
  struct obj_ptr {};
  struct obj_id {};
  typedef boost::bimap<boost::bimaps::tagged<const void*, obj_ptr>,
    boost::bimaps::tagged<int, obj_id>> PtrIdBimap;
  PtrIdBimap ptr_int_map_;
  int obj_id_ = 0;

  void remove(std::array<int, 2>& _link);
  void add_to_chain(int _start, std::vector<const void*>& _chain);
public:
  void add_link(const void* _obj_a, const void* _obj_b)
  {
    if (_obj_a == _obj_b)
      return;
    auto get_id = [this](const void* _obj)
    {
      int id = -1;
      auto pos = ptr_int_map_.by<obj_ptr>().find(_obj);
      if (pos != ptr_int_map_.by<obj_ptr>().end())
        id = (*pos).get_right();
      else
        ptr_int_map_.insert(PtrIdBimap::value_type(_obj, id = obj_id_++));
      return id;
    };
    auto id_a = get_id(_obj_a);
    auto id_b = get_id(_obj_b);

    links_.insert({ id_a, id_b });
    links_.insert({ id_b, id_a });
    computed_ = false;
  }
  void compute();
  const void* get_chain_element(size_t _ch_ind, size_t _elem_pos) const;
  size_t get_chain_number() const;
  size_t get_chain_element_number(size_t _ch_ind) const;
};

void GraphBase::Impl::remove(std::array<int, 2>& _link)
{
  links_.erase(_link);
  std::swap(_link[0], _link[1]);
  links_.erase(_link);
  std::swap(_link[0], _link[1]);
}

void GraphBase::Impl::add_to_chain(int _start, std::vector<const void*>& _chain)
{
  auto add_element = [this, &_chain](int _id)
  {
    auto el = ptr_int_map_.by<obj_id>().find(_id)->get<obj_ptr>();
    _chain.push_back(el);
  };
  add_element(_start);
  for (;;)
  {
    auto iter = links_.lower_bound({ _start , 0 });
    if (iter == links_.end())
      return;
    size_t match_nmbr = 0;
    auto next = *iter;
    for (; iter != links_.end() && (*iter)[0] == _start; ++iter)
      ++match_nmbr;

    if (match_nmbr != 1)
      return;

    add_element(next[1]);
    remove(next);
    _start = next[1];
  }
}

void GraphBase::Impl::compute()
{
  if (computed_)
    return;
  computed_ = true;
  while (!links_.empty())
  {
    auto start = *links_.begin();
    remove(start);
    std::vector<const void*> chain;
    add_to_chain(start[0], chain);
    std::reverse(chain.begin(), chain.end());
    add_to_chain(start[1], chain);
    chain_list_.push_back(std::move(chain));
  }
}

const void* GraphBase::Impl::get_chain_element(size_t _ch_ind, size_t _elem_pos) const
{
  return chain_list_[_ch_ind][_elem_pos];
}

size_t GraphBase::Impl::get_chain_number() const
{
  return chain_list_.size();
}

size_t GraphBase::Impl::get_chain_element_number(size_t _ch_ind) const
{
  if (_ch_ind >= chain_list_.size())
    return 0;
  return chain_list_[_ch_ind].size();
}

GraphBase::GraphBase() : impl_(new GraphBase::Impl) {}
GraphBase::~GraphBase() { delete impl_; }

void GraphBase::add_link(const void* _obj_a, const void* _obj_b)
{
  impl_->add_link(_obj_a, _obj_b);
}

const void* GraphBase::get_chain_element(size_t _ch_ind, size_t _elem_pos)
{
  return impl_->get_chain_element(_ch_ind, _elem_pos);
}

size_t GraphBase::get_chain_number() const
{
  return impl_->get_chain_number();
}

size_t GraphBase::get_chain_element_number(size_t _ch_ind) const
{
  return impl_->get_chain_element_number(_ch_ind);
}

void GraphBase::compute()
{
  impl_->compute();
}


} // namespace
