#include "Catch/catch.hpp"

#include "topology_help.hh"

#include <Topology/impl.hh>
#include <Topology/persistence.hh>

#include <fstream>

TEST_CASE("saveload1", "[PERS]")
{
  Topo::Wrap<Topo::Type::BODY> body = UnitTest::make_cube(UnitTest::cube_00);
  {
    std::ofstream oo("tmp.3++");
    auto sav = Topo::ISaver::make(oo);
    sav->save(body.get());
  }
  Topo::Wrap<Topo::Type::BODY> body1;
  {
    std::ifstream ii("tmp.3++");
    auto lod = Topo::ILoader::make(ii);
    auto obj = lod->load();
    REQUIRE(obj->sub_type() == Topo::SubType::BODY);
    body1.reset(static_cast<Topo::E<Topo::Type::BODY>*>(obj));
  }
}