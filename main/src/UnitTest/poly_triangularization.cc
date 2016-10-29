#include "catch/catch.hpp"

#include <PolygonTriangularization/poly_triang.hh>

#include <fstream>

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
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0, 0, 0 });
  plgn.push_back({ 0, 1, 0 });
  plgn.push_back({ 1, 1, 0 });
  plgn.push_back({ 1, 0, 0 });
  plgn.push_back({ 0.5, 0.5, 0 });
  ptg->add(plgn);
  auto& tris = ptg->triangles();
  REQUIRE(tris.size() == 3);
  REQUIRE(ptg->area() == 0.75);
  write_obj("1", plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "2"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 50,   0, 0 });
  plgn.push_back({ 25,  45, 0 });
  plgn.push_back({ -25,  45, 0 });
  plgn.push_back({ 0,  25, 0 });
  plgn.push_back({ -50,   0, 0 });
  plgn.push_back({ -25, -45, 0 });
  plgn.push_back({ 25, -45, 0 });
  plgn.push_back({ 0, -25, 0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(ptg->area() == 5125);
  write_obj("2", plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "3"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0,    0,  0 });
  plgn.push_back({ 1,    0,  0 });
  plgn.push_back({ 1,    1, 0 });
  plgn.push_back({ 1.25, 0, 0 });
  plgn.push_back({ 2,    0, 0 });
  plgn.push_back({ 2,    2, 0 });
  plgn.push_back({ 0,    2, 0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(ptg->area() == 3.875);
  write_obj("3", plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "4"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0,   1.5, 0 });
  plgn.push_back({ 1,   0,   0 });
  plgn.push_back({ 1,   0.5, 0 });
  plgn.push_back({ 1.5, 0,   0 });
  plgn.push_back({ 2,   0,   0 });
  plgn.push_back({ 2,   2,   0 });
  plgn.push_back({ 0,   2,   0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(ptg->area() == 3.125);
  write_obj(TEST_NAME, plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "5"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
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

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  write_obj(TEST_NAME, plgn, tris);
  REQUIRE(ptg->area() == 13.5);
  REQUIRE(tris.size() == plgn.size() - 2);
}


#undef TEST_NAME
#define TEST_NAME "6"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;

  plgn.push_back({ 0, 0, 0 });
  plgn.push_back({ 1, 0, 0 });
  plgn.push_back({ 1, 1, 0 });
  plgn.push_back({ 0, 2, 0 });
  plgn.push_back({ -1, 2, 0 });
  plgn.push_back({ -2, 0, 0 });
  plgn.push_back({ -2,-2, 0 });
  plgn.push_back({ 0,-4, 0 });
  plgn.push_back({ 0,-3, 0 });
  plgn.push_back({ -1,-2, 0 });
  plgn.push_back({ -1,-1, 0 });
  plgn.push_back({ -1.5, 0, 0 });
  plgn.push_back({ -1, 0, 0 });
  plgn.push_back({ 0, 1, 0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  write_obj(TEST_NAME, plgn, tris);
  REQUIRE(ptg->area() == 7.25);
  REQUIRE(tris.size() == plgn.size() - 2);
}


#undef TEST_NAME
#define TEST_NAME "7"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();

  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0, 0, 0 });
  plgn.push_back({ 3, 0, 0 });
  plgn.push_back({ 3, 3, 0 });
  plgn.push_back({ 0, 3, 0 });
  ptg->add(plgn);

  plgn.clear();
  plgn.push_back({ 1, 1, 0 });
  plgn.push_back({ 2, 1, 0 });
  plgn.push_back({ 2, 2, 0 });
  plgn.push_back({ 1, 2, 0 });
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  write_obj(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(ptg->area() == 8);
  REQUIRE(tris.size() == 8);
}

#undef TEST_NAME
#define TEST_NAME "8"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  {
    std::vector<Geo::Vector3> plgn =
    {
      { 1, 0, 0 },
      { 2, 1, 0 },
      { 1, 2, 0 },
      { 2, 3, 0 },
      { 1, 4, 0 },
      { 2, 5, 0 },
      { 1, 6, 0 },
      {-1, 6, 0 },
      {-2, 5, 0 },
      {-1, 4, 0 },
      {-2, 3, 0 },
      {-1, 2, 0 },
      {-2, 1, 0 },
      {-1, 0, 0 }
    };
    ptg->add(plgn);
  }
  {
    std::vector<Geo::Vector3> plgn =
    {
      { -0.5,  0.5, 0 },
      {  0.5,  0.5, 0 },
      {  0.5,  1.5, 0 },
      { -0.5,  1.5, 0 }
    };
    ptg->add(plgn);
  }
  {
    std::vector<Geo::Vector3> plgn =
    {
      { -0.5,  3.5, 0 },
      {  0.5,  3.5, 0 },
      {  0.5,  4.5, 0 },
      { -0.5,  4.5, 0 }
    };
    ptg->add(plgn);
  }

  auto& tris = ptg->triangles();
  write_obj(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 24);
  REQUIRE(ptg->area() == 16);
}

#undef TEST_NAME
#define TEST_NAME "9"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  {
    std::vector<Geo::Vector3> plgn =
    {
      { 0, 0, 0 },
      { 1, 0, 0 },
      { 2, 0, 0 },
      { 2, 2, 0 },
      { 0, 2, 0 },
      { 0, 1, 0 },
    };
    ptg->add(plgn);
  }

  auto& tris = ptg->triangles();
  write_obj(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 4);
  REQUIRE(ptg->area() == 4);
}

#undef TEST_NAME
#define TEST_NAME "PolyDifficult"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  {
    std::vector<Geo::Vector3> plgn =
    {
      {0.0025646612660641431, 0.12227424853876626, -0.31060595040406735 },
      {0.0019724747049671056, 0.12182496476650166, -0.31096316823484083 },
      {0.0012947238997314456, 0.12131084319736662, -0.31137207728228733 },
      {0.0011478593544021727, 0.12119940609339688, -0.31146065589753130 },
      {0.00068746477821405032, 0.12070482083939314, -0.31159553171530208},
      {0.00000000000000000, 0.13472500000000001, -0.32630700000000001   }
    };
    ptg->add(plgn);
  }

  auto& tris = ptg->triangles();
  write_obj(TEST_NAME, ptg->polygon(), tris);
}

