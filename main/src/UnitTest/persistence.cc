#include "Catch/catch.hpp"

#include "topology_help.hh"

#include <Topology/impl.hh>
#include <Topology/iterator.hh>
#include <Topology/persistence.hh>

#include <fstream>

TEST_CASE("saveload1", "[PERS]")
{
  Topo::Wrap<Topo::Type::BODY> body0 = UnitTest::make_cube(UnitTest::cube_00);
  {
    std::ofstream oo("tmp.3++");
    auto sav = Topo::ISaver::make(oo);
    sav->save(body0.get());
  }
  Topo::Wrap<Topo::Type::BODY> body1;
  {
    std::ifstream ii("tmp.3++");
    auto lod = Topo::ILoader::make(ii);
    auto obj = lod->load();
    REQUIRE(obj->sub_type() == Topo::SubType::BODY);
    body1.reset(static_cast<Topo::E<Topo::Type::BODY>*>(obj.get()));
  }
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it0(body0);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it1(body1);
  REQUIRE(bv_it0.size() == bv_it1.size());
  for (size_t i = 0; i < bv_it0.size(); ++i)
  {
    auto v0 = bv_it0.get(i);
    auto v1 = bv_it1.get(i);
    Geo::Point pt0, pt1;
    v0->geom(pt0);
    v1->geom(pt1);
    REQUIRE(pt0 == pt1);
  }
}