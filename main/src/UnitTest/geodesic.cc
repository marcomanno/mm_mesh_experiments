#include "catch/catch.hpp"

#include "Offset/geodesic.hh"
#include "Import/import.hh"
#include <Topology/iterator.hh>

#define MESH_FOLDER "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/"

TEST_CASE("mod_square", "[Geodesic]")
{
  auto mesh = IO::load_obj(MESH_FOLDER"mod_square.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(mesh);
  auto geod = Offset::IGeodesic::make();
  geod->compute(bv.get(0));
}

TEST_CASE("mod_square_little", "[Geodesic]")
{
  auto mesh = IO::load_obj(MESH_FOLDER"mod_square_little.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(mesh);
  auto geod = Offset::IGeodesic::make();
  geod->compute(bv.get(0));
}
