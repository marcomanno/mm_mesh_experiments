#include "Catch/catch.hpp"
#include <Topology/impl.hh>
#include <Topology/split_chain.hh>

TEST_CASE("split_face_00", "[SPLITCHAIN]")
{
  Topo::VertexChain vs(4);
  for (auto& v : vs)
    v.make<Topo::EE<Topo::Type::VERTEX>>();
  vs[0]->set_geom({ 0, 0, 0});
  vs[1]->set_geom({ 1, 0, 0 });
  vs[2]->set_geom({ 1, 1, 0 });
  vs[3]->set_geom({ 0, 1, 0 });
  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs);
  spl_ch->add_connection(vs[0], vs[2]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 2);
  REQUIRE(spl_ch->boundaries()[0].size() == 3);
  REQUIRE(spl_ch->boundaries()[1].size() == 3);
}
