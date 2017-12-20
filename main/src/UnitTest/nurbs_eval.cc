
#include "Catch/catch.hpp"

#include "boost/math/tools/polynomial.hpp"

#include "Geo/evalnurbs.hh"
#include "Geo/geo_function.hh"

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
  std::cout << "Eval nurmbs" << result[0] << " " << result[1] << "\n";
  
  // Result is polynomial x - 2 
  REQUIRE( result[0][0] == 2);
  REQUIRE( result[0][1] == 1);
  
  // Derivative is polynomial 1
  REQUIRE( result[1][0] == 1 );

  std::array<double, 2> eval_par1{-1, 1};
  mypoly t1(eval_par1.data(), unsigned(eval_par1.size() - 1)); // Evaluate at polynomial x - 1
  my_nrb.eval(t1, std::begin(result), std::end(result), &tspan);
  std::cout << "Eval nurmbs" << result[0] << " " << result[1] << "\n";

  // Result is polynomial 1 * (x - 1) + 1 
  REQUIRE( result[0][0] == 1);
  REQUIRE( result[0][1] == 1);
  
  // Derivative is polynomial 1
  REQUIRE( result[1][0] == 1 );
}

TEST_CASE("Curve00", "[NURBS]")
{
  using MyPt = Geo::Vector<double, 2>;
  std::vector<MyPt> pts = { {0,0}, {1,2} };
  std::vector<double> knots = { 0, 1 };

  auto crv = Geo::make_nurbs_curve<2>(pts, knots);
  MyPt pt;
  crv->evaluate({ 0 }, pt);
  REQUIRE((pt[0] == 0 && pt[1] == 0));
  std::vector<Geo::Curve<2>::Derivative> ders(2);
  ders[0].der_order_[0] = 1;
  ders[1].der_order_[0] = 3;
  crv->evaluate({ 0.5 }, pt, &ders);
  REQUIRE((pt[0] == 0.5 && pt[1] == 1));
  REQUIRE((ders[0].val_[0] == 1 && ders[0].val_[1] == 2));
  REQUIRE((ders[1].val_[0] == 0 && ders[1].val_[1] == 0));
  crv->evaluate({ 1 }, pt);
  REQUIRE((pt[0] == 1 && pt[1] == 2));

  using MyPt3 = Geo::Vector<double, 3>;
  std::vector<MyPt3> pts3 = { { 0,0,0 },{ 1,2,3},{ 2,0,0 } };
  std::vector<double> knots3 = { 0, 1, 2 };
  auto crv3 = Geo::make_nurbs_curve<3>(pts3, knots3);
  MyPt3 pt3;
  std::vector<Geo::Curve<3>::Derivative> ders3(1);
  crv3->evaluate({ 1 }, pt3, &ders3);
  REQUIRE((ders3[0].val_ == MyPt3{1, 2, 3}));
  crv3->evaluate({ 1 }, pt3, &ders3, true);
  REQUIRE((ders3[0].val_ == MyPt3{ 1, -2, -3 }));
}
