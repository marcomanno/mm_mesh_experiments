
#include "Catch/catch.hpp"

#include "boost/math/tools/polynomial.hpp"

#include "Geo/evalnurbs.hh"

#include <iostream>
#include <vector>
#include <array>

typedef boost::math::tools::polynomial<double> mypoly;

namespace Geo
{
template <> double convert<double, mypoly>(const mypoly & pt)
{
  return -pt[0];
}
}//namespace Geo

TEST_CASE("Evaluate nurbs", "[NURBS]" )
{
  typedef double Pt;

  std::vector<double> knots{0., 1.};
  std::vector<Pt> points{2, 3};
  Geo::Nub<Pt, double, mypoly> my_nrb;
  REQUIRE( my_nrb.init(points, knots) == true ); // Nurb (2 * (1-t) + 3 * t)
  mypoly result[2];
  
  std::array<double, 2> eval_par = {0, 1};

  mypoly tt(eval_par.data(), unsigned(eval_par.size() - 1)); // Evaluate at polynomial x
  const double tspan = 0.3;
  my_nrb.eval(tt, std::begin(result), std::end(result), &tspan);
  std::cout << result[0] << " " << result[1] << "\n";
  
  // Result is polynomial x - 2 
  REQUIRE( result[0][0] == 2);
  REQUIRE( result[0][1] == 1);
  
  // Derivative is polynomial 1
  REQUIRE( result[1][0] == 1 );

  std::array<double, 2> eval_par1{-1, 1};
  mypoly t1(eval_par1.data(), unsigned(eval_par1.size() - 1)); // Evaluate at polynomial x - 1
  my_nrb.eval(t1, std::begin(result), std::end(result), &tspan);
  std::cout << result[0] << " " << result[1] << "\n";

  // Result is polynomial 1 * (x - 1) + 1 
  REQUIRE( result[0][0] == 1);
  REQUIRE( result[0][1] == 1);
  
  // Derivative is polynomial 1
  REQUIRE( result[1][0] == 1 );
}