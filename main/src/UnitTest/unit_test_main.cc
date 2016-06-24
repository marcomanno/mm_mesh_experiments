#define CATCH_CONFIG_MAIN 
#include "catch/catch.hpp"

#include <PolygonTriangularization/poly_triang.hh>

static void write_obj(const char* _flnm,
  const std::vector<Geo::Vector3>& _plgn,
  const std::vector<std::array<size_t, 3>>& _tris)
{
  std::ofstream ff(std::string(_flnm) + ".obj");
  for (const auto& v : _plgn)
  {
    ff << "v" << v << "\n";
  }
  for (auto f : _tris)
  {
    for (auto& fi : f)
      fi += 1;
    ff << "f" << f << "\n";
  }

}

TEST_CASE("1", "[PolyTriang]") {
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
  write_obj("1", plgn, tris);
}

TEST_CASE("2", "[PolyTriang]") {
  auto pf = PolygonFil::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({  50,   0, 0 });
  plgn.push_back({  25,  45, 0 });
  plgn.push_back({ -25,  45, 0 });
  plgn.push_back({   0,  25, 0 });
  plgn.push_back({ -50,   0, 0 });
  plgn.push_back({ -25, -45, 0 });
  plgn.push_back({  25, -45, 0 });
  plgn.push_back({   0, -25, 0 });

  pf->init(plgn);
  auto& tris = pf->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  write_obj("2", plgn, tris);
}

TEST_CASE("3", "[PolyTriang]") {
  auto pf = PolygonFil::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({  0,    0,  0 });
  plgn.push_back({  1,    0,  0 });
  plgn.push_back({  1,    1, 0 });
  plgn.push_back({  1.25, 0, 0 });
  plgn.push_back({  2,    0, 0 });
  plgn.push_back({  2,    2, 0 });
  plgn.push_back({  0,    2, 0 });

  pf->init(plgn);
  auto& tris = pf->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  write_obj("3", plgn, tris);
}
