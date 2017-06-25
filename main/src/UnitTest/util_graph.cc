#include "Catch/catch.hpp"

#include "Utils/graph.hh"

#include "Base/basic_type.hh"

#include <iostream>
#include <memory>

template <class ObjectT>
static void print_graph(Utils::Graph<ObjectT>& _graph)
{
  for (size_t i = 0; i < _graph.get_chain_number(); ++i)
    for (size_t j = 0; j < _graph.get_chain_element_number(i); ++j)
      std::cout << "Graph " << *(_graph.get_chain_element(i, j)) << " ";
}

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
    my_graph.add_link(elems[i - 1].get(), elems[i].get());
  my_graph.compute();
  REQUIRE(my_graph.get_chain_number() == 1);
  REQUIRE(my_graph.get_chain_element_number(0) == ELEM_NMBR + 1);
  print_graph(my_graph);
  for (size_t i = 0; i < my_graph.get_chain_number(); ++i)
    for (size_t j = 0; j < my_graph.get_chain_element_number(i); ++j)
      REQUIRE(*(my_graph.get_chain_element(i, j)) == (j % ELEM_NMBR));
}

TEST_CASE("util_chain01", "[Utils]")
{
  typedef Base::BasicType<int> Obj;
  std::vector<std::unique_ptr<size_t>> elems;
  const int ELEM_NMBR = 4;
  for (int i = 0; i < ELEM_NMBR; ++i)
    elems.emplace_back(new size_t(i));
  Utils::Graph<size_t> my_graph;
  for (int i = 1; i < ELEM_NMBR; ++i)
    my_graph.add_link(elems[i - 1].get(), elems[i].get());
  my_graph.compute();
  print_graph(my_graph);
  REQUIRE(my_graph.get_chain_number() == 1);
  REQUIRE(my_graph.get_chain_element_number(0) == ELEM_NMBR);
  for (size_t i = 0; i < my_graph.get_chain_number(); ++i)
    for (size_t j = 0; j < my_graph.get_chain_element_number(i); ++j)
      REQUIRE(*(my_graph.get_chain_element(i, j)) == j);
}
