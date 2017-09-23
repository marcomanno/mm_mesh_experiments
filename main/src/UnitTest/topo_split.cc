#include "Catch/catch.hpp"
#include <Topology/impl.hh>
#include <Topology/split_chain.hh>

#include <initializer_list>

static Topo::VertexChain make_vertices(
  std::initializer_list<Geo::Point> _l)
{
  Topo::VertexChain vs;
  vs.reserve(_l.size());
  for (auto& pt : _l)
  {
    vs.emplace_back();
    vs.back().make<Topo::EE<Topo::Type::VERTEX>>();
    vs.back()->set_geom(pt);
  }
  return vs;
}

TEST_CASE("split_face_00", "[SPLITCHAIN]")
{
  Topo::VertexChain vs = make_vertices(
  { { 0, 0, 0 }, {1, 0, 0}, {1, 1, 0}, {0, 1, 0} } );
  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs);
  spl_ch->add_connection(vs[0], vs[2]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 2);
  REQUIRE(spl_ch->boundaries()[0].size() == 3);
  REQUIRE(spl_ch->boundaries()[1].size() == 3);
}

TEST_CASE("box_in_box", "[SPLITCHAIN]")
{
  Topo::VertexChain vs = make_vertices(
  { { 0, 0, 0 },{ 3, 0, 0 },{ 3, 3, 0 },{ 0, 3, 0 } });
  Topo::VertexChain vs1 = make_vertices(
  { { 1, 1, 0 },{ 2, 1, 0 },{ 2, 2, 0 },{ 1, 2, 0 } });

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs);
  spl_ch->add_connection(vs1[0], vs1[1]);
  spl_ch->add_connection(vs1[1], vs1[2]);
  spl_ch->add_connection(vs1[2], vs1[3]);
  spl_ch->add_connection(vs1[3], vs1[0]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 2);
  REQUIRE(spl_ch->boundaries()[0].size() == 4);
  REQUIRE(spl_ch->boundaries()[1].size() == 4);
}

/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.box_in_box2.jpg"/>


TEST_CASE("box_in_box2", "[SPLITCHAIN]")
{
  Topo::VertexChain vs0 = make_vertices(
  { { -2, -2, 0 },{ 2, 0, 0 },{ 2, 2, 0 },{ -2, 2, 0 } });
  Topo::VertexChain vs1 = make_vertices(
  { { -1, -1, 0 },{ 1, -1, 0 },{ 1, 1, 0 },{ -1, 1, 0 } });

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs0);
  spl_ch->add_connection(vs1[0], vs1[1]);
  spl_ch->add_connection(vs1[1], vs1[2]);
  spl_ch->add_connection(vs1[2], vs1[3]);
  spl_ch->add_connection(vs1[3], vs1[0]);

  spl_ch->add_connection(vs0[0], vs1[0]);
  spl_ch->add_connection(vs0[1], vs1[1]);
  spl_ch->add_connection(vs0[2], vs1[2]);
  spl_ch->add_connection(vs0[3], vs1[3]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 5);
}

/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.same_loop.jpg"/>
TEST_CASE("same_loop", "[SPLITCHAIN]")
{
  Topo::VertexChain vs0 = make_vertices(
  { { -4, 0, 0 },{ 4, 0, 0 },{ 0, 3, 0 } });
  Topo::VertexChain vs1 = make_vertices(
  { { -2, 1, 0 },{ -1, 1, 0 },{ 1, 1, 0 },{ 2, 1, 0 } });

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs0);
  spl_ch->add_connection(vs1[0], vs1[1]);
  spl_ch->add_connection(vs1[2], vs1[3]);

  spl_ch->add_connection(vs0[2], vs1[0]);
  spl_ch->add_connection(vs0[2], vs1[1]);
  spl_ch->add_connection(vs0[2], vs1[2]);
  spl_ch->add_connection(vs0[2], vs1[3]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 3);
  REQUIRE(spl_ch->boundary_islands(0) == nullptr);
  REQUIRE(spl_ch->boundary_islands(1) == nullptr);
  REQUIRE(spl_ch->boundary_islands(2) == nullptr);
}

/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.loop3.jpg"/>

TEST_CASE("loop3", "[SPLITCHAIN]")
{
  Topo::VertexChain vs0 = make_vertices(
  { { 1, 1, 0 },{ -1, 1, 0 } });
  Topo::VertexChain vs1 = make_vertices(
  { { -5, 0, 0 },{ 5, 0, 0 },{ 0, 3, 0 } });

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs1);
  spl_ch->add_connection(vs0[0], vs0[1]);
  spl_ch->add_connection(vs0[0], vs1[2]);
  spl_ch->add_connection(vs0[1], vs1[2]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 2);
  REQUIRE(spl_ch->boundary_islands(0) == nullptr);
  REQUIRE(spl_ch->boundary_islands(1) == nullptr);
}

TEST_CASE("loop4", "[SPLITCHAIN]")
{
  Topo::VertexChain vs0 = make_vertices(
  { { -1, 1, 0 },{ 1, 1, 0 } });
  Topo::VertexChain vs1 = make_vertices(
  { { -5, 0, 0 },{ 5, 0, 0 },{ 0, 3, 0 } });

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs1);
  spl_ch->add_connection(vs0[0], vs0[1]);
  spl_ch->add_connection(vs0[0], vs1[2]);
  spl_ch->add_connection(vs0[1], vs1[2]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 2);
  REQUIRE(spl_ch->boundary_islands(0) == nullptr);
  REQUIRE(spl_ch->boundary_islands(1) == nullptr);
}
/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.loop5.jpg"/>

TEST_CASE("loop5", "[SPLITCHAIN]")
{
  Topo::VertexChain vs0 = make_vertices(
  { { -3, -3, 0 }, { 3, -3, 0 },{ 3, 3, 0 },{ -3, 3, 0 } });
  Topo::VertexChain vs1 = make_vertices(
  { { -2, -2, 0 },{ 2, -2, 0 },{ 2, 2, 0 },{ -2, 2, 0 } });
  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vs0);

  spl_ch->add_connection(vs1[0], vs1[1]);
  spl_ch->add_connection(vs1[1], vs1[2]);
  spl_ch->add_connection(vs1[2], vs1[3]);
  spl_ch->add_connection(vs1[3], vs1[0]);
  spl_ch->add_connection(vs1[0], vs1[2]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 3);
  REQUIRE(spl_ch->boundary_islands(0)->size() == 1);
  REQUIRE((*spl_ch->boundary_islands(0))[0].size() == 4);
  REQUIRE(spl_ch->boundary_islands(1) == nullptr);
  REQUIRE(spl_ch->boundary_islands(2) == nullptr);
}
