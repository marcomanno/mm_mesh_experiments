
#include "catch/catch.hpp"
#include "Utils/equivalence_relation.hh"

#include <array>

TEST_CASE("EquivalentRelation_00", "[EquivalentRelation]")
{
  Utils::EquivalenceRelations<int> er;
  er.add_relation(1, 2);
  er.add_relation(3, 4);
  auto g0 = er.extract_equivalence_set();
  auto g1 = er.extract_equivalence_set();
  auto g3 = er.extract_equivalence_set();
  REQUIRE(g0.size() == 2);
  REQUIRE(g0[0] == 1);
  REQUIRE(g0[1] == 2);
  REQUIRE(g1.size() == 2);
  REQUIRE(g1[0] == 3);
  REQUIRE(g1[1] == 4);
  REQUIRE(g3.empty());
}

TEST_CASE("EquivalentRelation_01", "[EquivalentRelation]")
{
  Utils::EquivalenceRelations<int> er;
  er.add_relation(1, 2);
  er.add_relation(3, 4);
  er.add_relation(1, 3);
  auto g0 = er.extract_equivalence_set();
  auto g1 = er.extract_equivalence_set();
  auto g3 = er.extract_equivalence_set();
  REQUIRE(g0.size() == 4);
  REQUIRE(g0[0] == 1);
  REQUIRE(g0[1] == 2);
  REQUIRE(g0[2] == 3);
  REQUIRE(g0[3] == 4);
  REQUIRE(g1.size() == 0);
}

TEST_CASE("EquivalentRelation_02", "[EquivalentRelation]")
{
  Utils::EquivalenceRelations<int> er;
  std::array<int, 3> rels[] =
  {
    {0, 1, 1}, { 0, 2, 1 }, {0, 3, 1}, {1, 2, 0}
  };
  for (auto& r : rels)
  {
    auto val = er.add_relation(r[0], r[1]) ? 1 : 0;
    REQUIRE(val == r[2]);
  }
  auto g0 = er.extract_equivalence_set();
  REQUIRE(g0.size() == 4);
}
