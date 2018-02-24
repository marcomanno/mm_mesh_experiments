#include "catch/catch.hpp"

#include "Geo/polynomial_solver.hh"

#include <iostream>


TEST_CASE("Basic_pol_solver", "[POLYNOMIAL_SOLVER]")
{
  std::array<double, 3> poly2{1, 0, -1};
  auto roots = Geo::polygon_roots<2>(poly2);

  REQUIRE(roots.size() == 2);
  auto it_root = roots.begin();
  REQUIRE(*it_root == Approx(-1));
  REQUIRE(*++it_root == Approx(1));
}

TEST_CASE("Basic_pol_solver1", "[POLYNOMIAL_SOLVER]")
{
  std::array<double, 3> poly2{ 3., 10., -4. };
  auto roots = Geo::polygon_roots<2>(poly2);

  REQUIRE(roots.size() == 2);
  std::reverse(poly2.begin(), poly2.end());
  for (auto x : roots)
  {
    auto s = 0.;
    for (auto c : poly2)
      s = s * x + c;
    REQUIRE(s == Approx(0));
  }
}

