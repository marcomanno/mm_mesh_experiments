#include "Catch/catch.hpp"

#include <Topology/impl.hh>
#include <Topology/iterator.hh>
#include <Boolean/boolean.hh>
#include <Geo/vector.hh>

#include <functional>
#include <iostream>

namespace {
typedef std::function<double(size_t, size_t)> IndexToPoint;

double cube_00(size_t i, size_t _i_xyz) // Unit cube in (0, 1), (0, 1), (0, 1)
{
  return (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
};

double cube_01(size_t i, size_t _i_xyz) // Unit cube in (0, -1), (0, 1), (0, 1)
{
  auto coord = (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
  if (_i_xyz == 0)
    coord *= -1;
  return coord;
};

double cube_02(size_t i, size_t _i_xyz) // Unit cube in (-05, 05), (0, 1), (0, 1)
{
  auto coord = (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
  if (_i_xyz == 0)
    coord -= 0.5;
  return coord;
};

double cube_03(size_t i, size_t _i_xyz) // Unit cube in (-0.5, 0.5), (-0.5, 0.5), (0, 1)
{
  auto coord = (i & (size_t(1) << _i_xyz)) > 0 ? 1. : 0.;
  if (_i_xyz != 2)
    coord -= 0.5;
  return coord;
};

Topo::Wrap<Topo::Type::BODY> make_cube(IndexToPoint idx_to_pt)
{
  Topo::Wrap<Topo::Type::BODY> body;
  auto bb = body.make<Topo::EE<Topo::Type::BODY>>();

  Topo::Wrap<Topo::Type::VERTEX> verts[8];
  for (size_t i = 0; i < std::size(verts); ++i)
  {
    auto vert = verts[i].make<Topo::EE<Topo::Type::VERTEX>>();
    vert->set_geom({ idx_to_pt(i, 0), idx_to_pt(i, 1), idx_to_pt(i, 2) });
    vert->set_tolerance(1e-15);
  }

  size_t face_inds[][4] =
  {
    { 4, 5, 7, 6 },
    { 0, 2, 3, 1 },
    { 0, 1, 5, 4 },
    { 2, 6, 7, 3 },
    { 1, 3, 7, 5 },
    { 0, 4, 6, 2 }
  };

  Topo::Wrap<Topo::Type::FACE> faces[6];
  size_t f_idx = 0;
  for (auto& f : faces)
  {
    auto face = f.make<Topo::EE<Topo::Type::FACE>>();
    bb->insert_child(face);
    size_t * p = face_inds[f_idx++];
    for (size_t j = 0; j < 4; ++j)
    {
      auto& v = verts[p[j]];
      auto v_ptr = dynamic_cast<Topo::EE<Topo::Type::VERTEX>*>(v.get());
      face->insert_child(v_ptr);
    }
  }
  return body;
}

void print_body(Topo::Wrap<Topo::Type::BODY> _body)
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf;
  bf.reset(_body);
  for (size_t i = 0; i < bf.size(); ++i)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fe;
    fe.reset(bf.get(i));
    std::cout << "Face " << i << std::endl;
    for (size_t j = 0; j < fe.size(); ++j)
    {
      Geo::Point pt;
      fe.get(j)->geom(pt);
      std::cout << "Pt" << j << " = " << pt << std::endl;
    }
  }
}

}// namespace

TEST_CASE("make body", "[Topo]")
{
  Topo::Wrap<Topo::Type::BODY> body = make_cube(cube_00);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf;
  bf.reset(body);
  
  REQUIRE(bf.size() == 6);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be;
  be.reset(body);
  REQUIRE(be.size() == 12);
  for (auto i = be.size(); i-- > 0;)
  {
    auto edge = be.get(i);
    Geo::Segment seg;
    REQUIRE(edge->geom(seg) == true);
    REQUIRE(seg[0] != seg[1]);
    auto coordinate_match = [&seg]() -> bool
    {
      size_t match = 0;
      for (size_t i = 0; i < 3; ++i)
        match += seg[0][i] == seg[1][i];
      return match == 2;
    };
    REQUIRE(coordinate_match());
    
  }

  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv;
  bv.reset(body);
  REQUIRE(bv.size() == 8);
}

namespace
{
static Topo::Wrap<Topo::Type::BODY> body_1;
static Topo::Wrap<Topo::Type::BODY> body_2;

size_t edge_number_1()
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it;
  be_it.reset(body_1);
  return be_it.size();
}
size_t edge_number_2()
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it;
  be_it.reset(body_2);
  return be_it.size();
}

}// namespace

TEST_CASE("4 VV intersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_01);
  print_body(body_1);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::UNION);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv;
  bv.reset(result);
  REQUIRE(bv.size() == 12);
}

TEST_CASE("8 VE intersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_02);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  bool_solver->compute(Boolean::Operation::UNION);
}

TEST_CASE("4 EE intersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_03);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  bool_solver->compute(Boolean::Operation::UNION);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it;
  be_it.reset(body_1);
  REQUIRE(be_it.size() == 16);
  be_it.reset(body_2);
  REQUIRE(be_it.size() == 16);
}

