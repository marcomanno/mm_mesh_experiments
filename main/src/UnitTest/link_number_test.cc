#include "catch/catch.hpp"

#include "Geo/linking_number.hh"

#include <iostream>

#if 0
TEST_CASE("LnDisconnected", "[LINNUM]")
{
  std::vector<Geo::Vector<double, 3>> loop0 = {
    {0,0,0},
  {0,0,1},
  {0,1,1},
  {0,1,0}
  };
  std::vector<Geo::Vector<double, 3>> loop1 = {
    { 1,0,0 },
  { 1,0,1 },
  { 1,1,1 },
  { 1,1,0 }
  };
  auto res = Geo::LinkingNumber::compute(loop0, loop1);
  REQUIRE(res == 0);
}

TEST_CASE("Link1", "[LINNUM]")
{
  std::vector<Geo::Vector<double, 3>> loop1 = {
    { 0,0,0 },
  { 0,0,1 },
  { 0,1,1 },
  { 0,1,0 }
  };
  std::vector<Geo::Vector<double, 3>> loop0 = {
    { 0.5,0.5, -0.5 },
  { 0.5,0.5, 0.5 },
  { -0.5,0.5,0.5 },
  { -0.5, 0.5, -0.5 }
  };
  auto res = Geo::LinkingNumber::compute(loop0, loop1);
  REQUIRE(res == 0);
}

TEST_CASE("Link1_a", "[LINNUM]")
{
  std::vector<Geo::Vector<double, 3>> loop1 = {
    { 0,0,0 },
  { 0,0,1 },
  { 0,1,1 },
  { 0,1,0 }
  };
  std::vector<Geo::Vector<double, 3>> loop0 = {
    {  0.5, 0.5, -0.5 },
    {  0.0001, 0.5,  0 },
    {  0.5, 0.5,  0.5 },
    { -0.5, 0.5,  0.5 },
    { -0.5, 0.5, -0.5 }
  };
  auto res = Geo::LinkingNumber::compute(loop0, loop1);
  REQUIRE(res == 0);
}
#endif