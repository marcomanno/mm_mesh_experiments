#define CATCH_CONFIG_MAIN 
#include "catch/catch.hpp"

#include <PolygonTriangularization/poly_triang.hh>

TEST_CASE("Factorials are computed", "[factorial]") {
  auto pf = PolygonFil::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0, 0, 0 });
  plgn.push_back({ 0, 1, 0 });
  plgn.push_back({ 1, 1, 0 });
  plgn.push_back({ 1, 0, 0 });
  plgn.push_back({ 0.5, 0.5, 0 });
  pf->init(plgn);
  auto& tris = pf->triangles();
  REQUIRE(tris.size() == 3);
  std::ofstream ff("pippo.obj");
  for (const auto& v : plgn)
  {
    ff << "v" << v << "\n";
  }
  for (auto f : tris)
  {
    for (auto& fi : f)
      fi += 1;
    ff << "f" << f << "\n";
  }
}