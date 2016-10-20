#include "catch/catch.hpp"

#include "Geo/entity.hh"
#include <Geo/plane_fitting.hh>
#include "Geo/vector.hh"

#include <iostream>


TEST_CASE("FitPlane", "[Geo]")
{
  auto best_plane = Geo::IPlaneFit::make();
  best_plane->init(4);
  best_plane->add_point({ 0, 1 , 0 });
  best_plane->add_point({ 1, 0 , 0 });
  best_plane->add_point({ 1, 1 , 0 });
  best_plane->add_point({ -1, -1 , 0 });
  Geo::Point c, n;
  best_plane->compute(c, n);
  REQUIRE(Geo::same(c, Geo::Point{ 0.25, 0.25, 0 }, 0.));
  REQUIRE(Geo::same(n, Geo::Point{ 0, 0, 1 }, 0.));
}


TEST_CASE("FitPlane2", "[Geo]")
{
  auto best_plane = Geo::IPlaneFit::make();
  best_plane->init(8);
  best_plane->add_point({ 0, 1 , 0.01 });
  best_plane->add_point({ 1, 0 , 0.01 });
  best_plane->add_point({ 1, 1 , -0.01 });
  best_plane->add_point({ -1, -1 , -0.01 });
  Geo::Point c, n;
  best_plane->compute(c, n);
  REQUIRE(Geo::same(n, Geo::Point{ 0, 0, 1 }, 0.1));
}
