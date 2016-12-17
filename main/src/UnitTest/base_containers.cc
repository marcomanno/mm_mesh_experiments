#include "Catch/catch.hpp"
#include "Base/lookup_list.hh"
#include "Base/basic_type.hh"

#include <iostream>

TEST_CASE("LookupList1", "[BASE]")
{
  Base::LookupList<Base::BasicType<int>> aa;
  aa.emplace_back(Base::BasicType<int>(4));
  aa.emplace_back(Base::BasicType<int>(4));
  aa.emplace_back(Base::BasicType<int>(3));
  aa.emplace_back(Base::BasicType<int>(2));
  aa.emplace_back(Base::BasicType<int>(6));
  aa.emplace_back(Base::BasicType<int>(7));
  aa.emplace_back(Base::BasicType<int>(0));
  REQUIRE(aa.size() == 6);
  auto it = aa.lookup(0);
  aa.erase(it);
  it = aa.lookup(2);
  aa.erase(it);
  REQUIRE(aa.size() == 4);
  auto cit = aa.cbegin();
  REQUIRE(*cit == 4);
  REQUIRE(*++cit == 3);
  REQUIRE(*++cit == 6);
  REQUIRE(*++cit == 7);
  REQUIRE(++cit == aa.cend());
}
