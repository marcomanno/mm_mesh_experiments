#define CATCH_CONFIG_MAIN 
#include "catch/catch.hpp"

#include <PolygonTriangularization/poly_triang.hh>

static void write_obj(const char* _flnm,
  const std::vector<Geo::Vector3>& _plgn,
  const std::vector<std::array<size_t, 3>>& _tris)
{
  std::ofstream ff(std::string(_flnm) + ".obj");
  for (const auto& v : _plgn) { ff << "v" << v << "\n"; }
  for (auto f : _tris)
  {
    for (auto& fi : f)
      fi += 1;
    ff << "f" << f << "\n";
  }
}

#undef TEST_NAME
#define TEST_NAME "1"
TEST_CASE(TEST_NAME, "[PolyTriang]") {
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
  REQUIRE(pf->area() == 0.75);
  write_obj("1", plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "2"
TEST_CASE(TEST_NAME, "[PolyTriang]") {
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
  REQUIRE(pf->area() == 5125);
  write_obj("2", plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "3"
TEST_CASE(TEST_NAME, "[PolyTriang]") {
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
  REQUIRE(pf->area() == 3.875);
  write_obj("3", plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "4"
TEST_CASE(TEST_NAME, "[PolyTriang]") {
  auto pf = PolygonFil::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({  0,   1.5, 0 });
  plgn.push_back({  1,   0,   0 });
  plgn.push_back({  1,   0.5, 0 });
  plgn.push_back({  1.5, 0,   0 });
  plgn.push_back({  2,   0,   0 });
  plgn.push_back({  2,   2,   0 });
  plgn.push_back({  0,   2,   0 });

  pf->init(plgn);
  auto& tris = pf->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(pf->area() == 3.125);
  write_obj(TEST_NAME, plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "5"
TEST_CASE(TEST_NAME, "[PolyTriang]") {
  auto pf = PolygonFil::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0,   0,   0 });
  plgn.push_back({ 2,   0,   0 });
  plgn.push_back({ 2,   1,   0 });
  plgn.push_back({ 1,   1,   0 });
  plgn.push_back({ 2,   2,   0 });
  plgn.push_back({ 3,   2,   0 });
  plgn.push_back({ 3,   0,   0 });
  plgn.push_back({ 4,   0,   0 });
  plgn.push_back({ 4,   4,   0 });
  plgn.push_back({ 0,   4,   0 });

  pf->init(plgn);
  auto& tris = pf->triangles();
  write_obj(TEST_NAME, plgn, tris);
  REQUIRE(pf->area() == 13.5);
  REQUIRE(tris.size() == plgn.size() - 2);
}


