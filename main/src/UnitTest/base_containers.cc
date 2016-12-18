#include "Catch/catch.hpp"
#include "Base/lookup_list.hh"
#include "Base/basic_type.hh"

#include <iostream>
#include <string>

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

TEST_CASE("LookupList2", "[BASE]")
{
  Base::LookupList<std::string> aa;
  aa.emplace_back("22");
  aa.emplace_back("bbb");
  aa.emplace_back("aaa");
  aa.emplace_back("zzz");
  REQUIRE(aa.size() == 4);
  auto it = aa.lookup(std::string("22"));
  aa.erase(it);
  REQUIRE(aa.size() == 3);
  auto cit = aa.cbegin();
  REQUIRE(*cit == "bbb");
  REQUIRE(*++cit == "aaa");
  REQUIRE(*++cit == "zzz");
  REQUIRE(++cit == aa.cend());
}
