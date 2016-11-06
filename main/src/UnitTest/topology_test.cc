#include "Catch/catch.hpp"

#include "topology_help.hh"

#include <Topology/iterator.hh>
#include <Boolean/boolean.hh>
#include <Geo/vector.hh>
#include <Import/import.hh>

using namespace UnitTest;

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

  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/result.obj", result);

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
  auto result = bool_solver->compute(Boolean::Operation::UNION);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 14);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 28);
}

TEST_CASE("4 EE intersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_03);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);

  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/result.obj", result);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 8);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 18);
}

TEST_CASE("3 FF intersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_04);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);

  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/result.obj", result);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 9);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 21);
}

TEST_CASE("pyramid", "[Bool]")
{
  auto pyr1 = IO::load_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/pyramid.obj");
  auto pyr2 = IO::load_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/pyramid.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(pyr2);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] = -pt[2];
    x->set_geom(pt);
  }
  // !!! Attention, pyr2 is inverted.
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::INTERSECTION);
  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/result.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 9);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 16);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv1_it(result);
  REQUIRE(bv1_it.size() == 9);
}

TEST_CASE("2apple", "[Bool]")
{
  const char* out_flnm = "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/2apple_";
  for (auto dz : { 2.5, 2., 1.5, 1. })
  {
    auto pyr1 = IO::load_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/Apple_00.obj");
    auto pyr2 = IO::load_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/Apple_00.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(pyr2);
    for (auto& x : bv_it)
    {
      Geo::Point pt;
      x->geom(pt);
      pt[2] += dz;
      x->set_geom(pt);
    }
    IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/apple_moved.obj", pyr2);
    auto bool_solver = Boolean::ISolver::make();
    bool_solver->init(pyr1, pyr2);
    auto result = bool_solver->compute(Boolean::Operation::UNION);
    std::string flnm(out_flnm);
    flnm += std::to_string(dz) + ".obj";
    IO::save_obj(flnm.c_str(), result);
  }
}

#if 0
TEST_CASE("2tuna", "[Bool]")
{
  auto pyr1 = IO::load_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA.obj");
  auto pyr2 = IO::load_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(pyr2);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] += 0.2;
    x->set_geom(pt);
  }
  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA_moved.obj", pyr2);
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/tua_result.obj", result);
}
#endif