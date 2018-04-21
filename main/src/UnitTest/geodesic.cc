#include "catch/catch.hpp"

#include "Offset/geodesic.hh"
#include "Import/import.hh"
#include <Topology/iterator.hh>
#include <fstream>

#define MESH_FOLDER "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/"

static void geodesic_test(const char* _flnm, 
                          const std::initializer_list<double>& _dists)
{
  auto mesh = IO::load_obj((std::string(MESH_FOLDER) + _flnm + ".obj").c_str());
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(mesh);
  auto geod = Offset::IGeodesic::make();
  geod->compute(bv.get(0));
  for (auto dist : _dists)
  {
    std::vector<Geo::VectorD3> pts;
    std::vector<std::array<size_t, 2>> inds;
    geod->find_graph(dist, pts, inds);
    std::string flnm(_flnm);
    std::ofstream ff(flnm + "_" + std::to_string(dist) + ".obj");
    ff << std::setprecision(17);
    for (const auto& v : pts) { ff << "v" << v << "\n"; }
    for (const auto& v : pts) { ff << "v" << v << "\n"; }
    for (const auto& f : inds)
    {
      ff << "f " << f[0] + 1 << " " << f[1] + 1 << " " << f[0] + 1 + pts.size() << "\n";
    }
  }
}

TEST_CASE("mod_square", "[Geodesic]")
{
  auto mesh = IO::load_obj(MESH_FOLDER"mod_square.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(mesh);
  auto geod = Offset::IGeodesic::make();
  geod->compute(bv.get(0));
}

TEST_CASE("mod_square_little", "[Geodesic]")
{
  geodesic_test("mod_square_little", { 0.2, 0.8, 1.2, 2., 2.5 });
}

TEST_CASE("ciminiera", "[Geodesic]")
{
  geodesic_test("ciminiera", { 0.3, 0.8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 });
}

TEST_CASE("ciminiera_01", "[Geodesic]")
{
  geodesic_test("ciminiera_01", { 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 });
}

TEST_CASE("ciminiera_02", "[Geodesic]")
{
  geodesic_test("ciminiera_02", { 4 });
}

TEST_CASE("ciminiera_03", "[Geodesic]")
{
  geodesic_test("ciminiera_03", { 1,2,3,4,5,6,7,7.3 });
}

TEST_CASE("ciminiera_04", "[Geodesic]")
{
  geodesic_test("ciminiera_04", { 4, 4.5,5, 5.5, 6,6.5, 7, 7.5, 8 });
}

TEST_CASE("ciminiera_05", "[Geodesic]")
{
  geodesic_test("ciminiera_05", { 7, 7.1, 7.2, 7.3, 7.4, 7.5, 8, 8.1, 8.2 });
}

TEST_CASE("sphere_10", "[Geodesic]")
{
  geodesic_test("sphere_10", { 0.2, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.5, 2, 2.5, 3 });
}

TEST_CASE("sphere_10_p0", "[Geodesic]")
{
  geodesic_test("sphere_10_p0",{ 0.2, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.5, 2, 2.5, 3 });
}

TEST_CASE("sphere_10_p1", "[Geodesic]")
{
  geodesic_test("sphere_10_p1", { 0.2, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.5, 2, 2.5, 3 });
}

TEST_CASE("sphere_10_p2", "[Geodesic]")
{
  geodesic_test("sphere_10_p2", { 0.2, 0.5, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.5, 3 });
}