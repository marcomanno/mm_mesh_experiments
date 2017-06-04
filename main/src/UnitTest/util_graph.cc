#include "Catch/catch.hpp"

#include "Utils/graph.hh"

#include "Base/basic_type.hh"

#include <memory>

TEST_CASE("util_chain00", "[Utils]")
{
  typedef Base::BasicType<int> Obj;
  std::vector<std::unique_ptr<int>> elems;
  const int ELEM_NMBR = 4;
  for (int i = 0; i < ELEM_NMBR; ++i)
    elems.emplace_back(new int(i));
  Utils::Graph<int> my_graph;
  my_graph.add_link(elems[ELEM_NMBR - 1].get(), elems[0].get());
  for (int i = 1; i < ELEM_NMBR; ++i)
    my_graph.add_link(elems[i].get(), elems[i - 1].get());
  my_graph.compute();
  REQUIRE(my_graph.get_chain_number() == 1);
  REQUIRE(my_graph.get_chain_element_number(0) == ELEM_NMBR + 1);
}

