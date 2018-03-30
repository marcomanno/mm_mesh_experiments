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
  geodesic_test("ciminiera", { 0.3, 0.8 });
}

