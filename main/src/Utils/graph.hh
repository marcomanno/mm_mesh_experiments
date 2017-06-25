
namespace Utils {

// Given a set of links between elements organize them in chains.
struct GraphBase
{
  template <class ObjectT> friend struct Graph;
  void compute();
  const void* get_chain_element(size_t _ch_ind, size_t _elem_pos);
  size_t get_chain_number() const;
  size_t get_chain_element_number(size_t _ch_ind) const;
private:
  GraphBase();
  ~GraphBase();
  void add_link(const void* _obj_a, const void* _obj_b);
  class Impl;
  Impl* impl_;
};

template <class ObjectT> struct Graph : public GraphBase
{
  void add_link(const ObjectT* _obj_a, const ObjectT* _obj_b)
  {
    GraphBase::add_link(_obj_a, _obj_b);
  }
  ObjectT* get_chain_element(size_t _ch_ind, size_t _elem_pos)
  {
    return const_cast<ObjectT*>(static_cast<const ObjectT*>(GraphBase::get_chain_element(_ch_ind, _elem_pos)));
  }
};

} // namespace
