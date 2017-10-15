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

/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.loop6.jpg"/>

TEST_CASE("loop6", "[SPLITCHAIN]")
{
  std::initializer_list<Geo::Point> pts =
  { { 0, 0, 0 },{ 0, 4, 0 },
    { -5, 2, 0 },{ -3, 2, 0 },{ -2, 2, 0 },{ -1, 2, 0 },
    { 1, 2, 0 },{ 2, 2, 0 },{ 3, 2, 0 },{ 5, 2, 0 } };

  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;
  for (auto& pt : pts)
  {
    verts.emplace_back();
    verts.back().make<Topo::EE<Topo::Type::VERTEX>>();
    verts.back()->set_geom(pt);
  }
  Topo::VertexChain bndr;
  for (auto i : {0,9,1,2})
    bndr.push_back(verts[i]);

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(bndr);

  for (auto i : { 3,4,5,6,7,8 })
    for (auto j : { 0,1 })
      spl_ch->add_connection(verts[i], verts[j]);

  spl_ch->add_connection(verts[0], verts[1]);

  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 8);
  for (auto i = spl_ch->boundaries().size(); i-- > 0;)
    REQUIRE(spl_ch->boundary_islands(i) == nullptr);
}

/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.loop7.jpg"/>

TEST_CASE("loop7", "[SPLITCHAIN]")
{
  std::initializer_list<Geo::Point> pts =
  { { -3, 3, 0 },{ 3, 3, 0 },{ -3, -3, 0 },{ 3, -3, 0 },
  { -1, 0, 0 },{ 0, 1, 0 },{ 1, 0, 0 },{ 0, -1, 0 },
  { -2, 2, 0 },{ 0, 0, 0 } };

  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;
  for (auto& pt : pts)
  {
    verts.emplace_back();
    verts.back().make<Topo::EE<Topo::Type::VERTEX>>();
    verts.back()->set_geom(pt);
  }
  Topo::VertexChain bndr;
  for (auto i : { 2,3,1,0,8,0 })
    bndr.push_back(verts[i]);

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(bndr);

  std::initializer_list<std::pair<size_t, size_t>> conns =
  { {8, 4},  {8, 5}, {4, 5}, {4,9}, {4,7}, {6,7}, {6,5} };
  for (auto i : conns)
    spl_ch->add_connection(verts[i.first], verts[i.second]);

  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 3);
  for (auto i = spl_ch->boundaries().size(); i-- > 0;)
    REQUIRE(spl_ch->boundary_islands(i) == nullptr);
}

/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.loop8.jpg"/>

TEST_CASE("loop8", "[SPLITCHAIN]")
{
  std::initializer_list<Geo::Point> pts =
  { { -5, -5, 0 },{ 5, -5, 0 },{ 5, 5, 0 },{ -5, 5, 0 },
  { -2, 3},
  { -1, 4, 0 },{ -1, 3, 0 },{ -1, 2, 0 },{ -1, 1, 0 },{ -1, -1, 0 },{ -1, -2, 0 },
  { -1, -4, 0 },{ -2, -4, 0 },{ -3, -4, 0 },{ -4, -4, 0 },
  { 3, 4, 0}, {3,-4,0},{4,4,0}, {4, -4, 0},{4, 0,0} };

  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;
  for (auto& pt : pts)
  {
    verts.emplace_back();
    verts.back().make<Topo::EE<Topo::Type::VERTEX>>();
    verts.back()->set_geom(pt);
  }
  Topo::VertexChain bndr;
  for (auto i : { 0, 1, 2,3 })
    bndr.push_back(verts[i]);

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(bndr);

  std::initializer_list<std::pair<size_t, size_t>> conns =
  {
    { 3, 4 },
    { 4, 5 },{ 4,6 },{ 4,7 },{ 4,8 },{ 4,9 },{ 4,10 },{ 4,11 },{ 4,12 },{ 4,13 },{ 4,14 },
    {5,6}, {7,8}, {9,10},{11,12}, {13,14},
    {19, 15},{ 19, 16 },{ 19, 17 },{ 19, 18 },
    {15,17},{16,18},{5,15}, {11,16},
    {19,2},{19,1}
  };
  for (auto i : conns)
    spl_ch->add_connection(verts[i.first], verts[i.second]);

  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 11);
  for (auto i = spl_ch->boundaries().size(); i-- > 0;)
    REQUIRE(spl_ch->boundary_islands(i) == nullptr);
}

/// <image url="$(SolutionDir)..\main\src\UnitTest\topo_split.cc.loop9.jpg"/>

TEST_CASE("loop9", "[SPLITCHAIN]")
{
  std::initializer_list<Geo::Point> pts =
  { { -5, 0, 0 },{ 5, 0, 0 },{ 5, 5, 0 },{ -5, 5, 0 },
  { -4, 1, 0 },{ -1, 1, 0 }, {-1, 4, 0}, {-4, 4, 0},
  { 0, 1, 0 },{ 4, 1, 0 },{ 4, 4, 0 },{ 0, 4, 0 },
  { -3, 2, 0 },{ -2, 2, 0 },{ -3, 3, 0 },
  { 1, 2, 0 },{ 2, 2, 0 },{ 3, 2, 0 }, { 1, 3, 0 },{ 3, 3, 0 } };

  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;
  for (auto& pt : pts)
  {
    verts.emplace_back();
    verts.back().make<Topo::EE<Topo::Type::VERTEX>>();
    verts.back()->set_geom(pt);
  }
  Topo::VertexChain bndr;
  for (auto i : { 0, 1, 2, 3 })
    bndr.push_back(verts[i]);

  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(bndr);

  std::initializer_list<std::pair<size_t, size_t>> conns =
  {
    { 4, 5 },{ 5, 6 },{ 6, 7 },{ 4, 7 },
    { 8, 9 },{ 9, 10},{ 10, 11 },{ 11, 8 },
    {6,11},
    {12,13},{13,14},{14,12},
    { 15,16 },{ 16, 17 },{ 17,19 },{15,18}, {16,18 }, {16,19} };
  for (auto i : conns)
    spl_ch->add_connection(verts[i.first], verts[i.second]);

  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 6);
  size_t islands_nmbr[] = {1,1,1,0,0,0};
  for (auto i = spl_ch->boundaries().size(); i-- > 0;)
  {
    auto islnd_i = spl_ch->boundary_islands(i);
    if (islnd_i == nullptr)
      REQUIRE(islands_nmbr[i] == 0);
    else
    REQUIRE(islnd_i->size() == islands_nmbr[i]);
  }
}


TEST_CASE("face_with_island", "[SPLITCHAIN]")
{
  auto bndr = make_vertices(
  { {-2, -2, 0}, {2, -2, 0}, {2, 2, 0}, {-2, 2, 0} } );
  auto isl = make_vertices(
  { { -1, -1, 0 }, { -1, 1, 0 }, { 1, 1, 0 }, { 1, -1, 0 } });
  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(bndr);
  spl_ch->add_chain(isl);
  spl_ch->add_connection(bndr[0], isl[0]);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 1);
  REQUIRE(spl_ch->boundaries()[0].size() == 10);
}


TEST_CASE("split_chain_bug_00", "[SPLITCHAIN]")
{
  int indeces[] = { 9, 4, 1, 5, 2, 3, 6, 7, 0, 10, 8 };
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> vvs(std::size(indeces));
  for (auto i : indeces)
    vvs[i].make<Topo::EE<Topo::Type::VERTEX>>();

  Geo::Point pts[] = {
    { 0.13212922914098529, 0.88317105433318699, -0.0047252294015959666 },
    { 0.14328539192534029, 0.88480892597998251, -0.016969278134127776 },
    { 0.14328600470000000, 0.88480901720000005, -0.016969976950000000 },
    { 0.13522943943115151, 0.87078071968850335, -0.0074214916210336886 },
    { 0.13522916573103783, 0.87078026098021999, -0.0074211900466691176 },
    { 0.13522900639999999, 0.87077999110000004, -0.0074210101739999998 },
    { 0.13522882403910802, 0.87078069475080844, -0.0074208654984565855 },
    { 0.13522876428273789, 0.87078094821520313, -0.0074208051851865795 },
    { 0.13522869843705265, 0.87078121692922950, -0.0074207449840044517 },
    { 0.13522864822691258, 0.87078141762363725, -0.0074207013176136281 },
    { 0.13212899859999999, 0.88317102189999996, -0.0047250059430000001 } };
  for (int i = 0; i < vvs.size(); ++i)
  {
    vvs[i]->set_geom(pts[i]);
  }
  auto spl_ch = Topo::ISplitChain::make();
  spl_ch->add_chain(vvs);
  Topo::Wrap<Topo::Type::VERTEX> vert;
  vert.make<Topo::EE<Topo::Type::VERTEX>>();
  vert->set_geom({ 0.13212927453559242, 0.88317087288674823, -0.0047252688801808270 });
  spl_ch->add_connection(vvs[6], vvs[4]);
  spl_ch->add_connection(vvs[8], vvs[0]);
  spl_ch->add_connection(vvs[0], vert);
  spl_ch->compute();
  REQUIRE(spl_ch->boundaries().size() == 3);

};