#include "Catch/catch.hpp"

#include "topology_help.hh"

#include <Topology/iterator.hh>
#include <Boolean/boolean.hh>
#include <Geo/vector.hh>
#include <Import/import.hh>

using namespace UnitTest;

#define MESH_FOLDER "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/"

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

TEST_CASE("4VVintersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_01);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::UNION);

  IO::save_obj("result_4VVintersections.obj", result);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv;
  bv.reset(result);
  REQUIRE(bv.size() == 12);
}

TEST_CASE("8VEintersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_02);
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::UNION);
  IO::save_obj("result_8VEintersections.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 14);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 28);
}

TEST_CASE("4EEintersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_03);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);

  IO::save_obj("result_4EEintersections.obj", result);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 8);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 18);
}
/// <image url="$(SolutionDir)..\main\src\UnitTest\bool4ff.jpg"/>

TEST_CASE("3FFintersections", "[Bool]")
{
  body_1 = make_cube(cube_00);
  body_2 = make_cube(cube_04);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);

  IO::save_obj("result_3FFintersections.obj", result);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 9);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 21);
}

TEST_CASE("InternalLoop", "[Bool]")
{
  body_1 = make_cube(cube_03);
  body_2 = IO::load_obj(MESH_FOLDER"pyramid.obj");

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);

  IO::save_obj("result_internal_loop.obj", result);

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 10);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 20);
}

namespace {

Topo::Wrap<Topo::Type::BODY> box_rot_test(
  std::function<Geo::Point(const Geo::Point&)> _transf)
{
  body_1 = make_cube(cube_04);
  body_2 = make_cube(cube_04);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(body_2);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    x->set_geom(_transf(pt));
  }
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::UNION);
  return result;
}

}

TEST_CASE("BoxRotation_1", "[Bool]")
{
  const double sa = sin(0.1), ca = cos(0.1);
  auto transf = [&sa, &ca](const Geo::Point& pt)
  {
    return Geo::Point{
      pt[0] * ca + pt[1] * sa,
      -pt[0] * sa + pt[1] * ca,
      pt[2] };
  };
  auto result = box_rot_test(transf);
  IO::save_obj("result_BoxRotation_1.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bf_it.size() == 34);
  REQUIRE(bv_it.size() == 32);
}

TEST_CASE("BoxRotation_2", "[Bool]")
{
  const double sa = sin(0.01), ca = cos(0.01);
  auto transf = [&sa, &ca](const Geo::Point& pt)
  {
    auto pt1 = Geo::Point{
      pt[0] * ca + pt[1] * sa,
      -pt[0] * sa + pt[1] * ca,
      pt[2] };
    return Geo::Point{
      pt1[0],
      pt[1] * ca + pt[2] * sa,
      -pt[1] * sa + pt[2] * ca};
  };
  auto result = box_rot_test(transf);
  IO::save_obj("result_BoxRotation_2.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bf_it.size() == 20);
  REQUIRE(bv_it.size() == 34);
}

TEST_CASE("pyramid", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"pyramid.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"pyramid.obj");
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
  IO::save_obj("result_pyramid.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 9);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::EDGE> be_it(result);
  REQUIRE(be_it.size() == 16);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv1_it(result);
  REQUIRE(bv1_it.size() == 9);
}

TEST_CASE("2apple", "[Bool]")
{
  std::vector<size_t> top_elem_nmbr;
  for (auto dz : { 2.5 , 2., 1.5, 1., 0.5 })
  {
    auto pyr1 = IO::load_obj(MESH_FOLDER"Apple_00.obj");
    auto pyr2 = IO::load_obj(MESH_FOLDER"Apple_00.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(pyr2);
    for (auto& x : bv_it)
    {
      Geo::Point pt;
      x->geom(pt);
      pt[2] += dz;
      x->set_geom(pt);
    }
    auto bool_solver = Boolean::ISolver::make();
    bool_solver->init(pyr1, pyr2);
    auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
    std::string flm("result_2apple_");
    flm += std::to_string(dz) + ".obj";
    IO::save_obj(flm.c_str(), result);
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv1_it(result);
    Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
    top_elem_nmbr.push_back(bv1_it.size());
    top_elem_nmbr.push_back(bf_it.size());
  }
  REQUIRE(top_elem_nmbr ==
    std::vector<size_t>({ 412, 396, 425, 395, 470, 415, 523, 449, 539, 460 }));
}

TEST_CASE("FaceSplit", "[Bool]")
{
  body_1 = make_cube(cube_04);
  body_2 = IO::load_obj(MESH_FOLDER"extr_tri.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(body_1, body_2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);

  IO::save_obj("result_FaceSplit.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 14);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 10);
}

TEST_CASE("FacePartialOverlap", "[Bool]")
{
  auto sol0 = IO::load_obj(MESH_FOLDER"test_overlap_0.obj");
  auto sol1 = IO::load_obj(MESH_FOLDER"test_overlap_1.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(sol0, sol1);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
  IO::save_obj("result_FacePartialOverlap.obj", result);
}

TEST_CASE("FacePartialOverlap2", "[Bool]")
{
  auto sol0 = IO::load_obj(MESH_FOLDER"overlap_1.obj");
  auto sol1 = IO::load_obj(MESH_FOLDER"overlap_2.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(sol0, sol1);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
  IO::save_obj("result_FacePartialOverlap2.obj", result);
}

TEST_CASE("tuna0.1", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(pyr2);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] += 0.1;
    x->set_geom(pt);
  }
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
  IO::save_obj("result_tuna_0.1.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it1(result);
  REQUIRE(bv_it1.size() == 2519);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 3814);
}

TEST_CASE("tuna0.2", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(pyr2);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] += 0.2;
    x->set_geom(pt);
  }
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
  IO::save_obj("result_tuna_0.2.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it1(result);
  REQUIRE(bv_it1.size() == 2657);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 4152);
}

TEST_CASE("tuna0.4", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(pyr2);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] += 0.4;
    x->set_geom(pt);
  }
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
  IO::save_obj("result_tuna_0.4.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it1(result);
  REQUIRE(bv_it1.size() == 2328);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 3942);
}

TEST_CASE("tuna_inv_sel", "[Bool]")
{
  auto sol_a = IO::load_obj(MESH_FOLDER"tuna inv_sela.obj");
  auto sol_b = IO::load_obj(MESH_FOLDER"tuna inv_selb.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(sol_a, sol_b);
  auto result = bool_solver->compute(Boolean::Operation::SPLITA);
  IO::save_obj("result_tuna_inv_sel.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it1(result);
  //REQUIRE(bv_it1.size() == 2328);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 3942);
}

TEST_CASE("tuna_p0", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"2tuna_p0a.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"2tuna_p0b.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_tuna_p0.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 57);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 60);
}


TEST_CASE("tuna_p1", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"aaaa0.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"aaaa1.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_tuna_p1.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 75);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 114);
}

TEST_CASE("tuna_p1a", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA_PIECE3_0_0.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA_PIECE3_1.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_tuna_p1a.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 41);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 51);
}

TEST_CASE("tuna_P4", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA_PIECE_4_0.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA_PIECE_4_1.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLITB);
  IO::save_obj("result_tuna_P4.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 31);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 20);
}

TEST_CASE("tuna_left", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA_left0.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA_left1.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_tuna_left.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 17);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 15);
}

TEST_CASE("tuna_p2", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA_p2a.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA_p2b.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_tuna_p2.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 52);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 67);
}

TEST_CASE("tuna_p3", "[Bool]")
{
	auto pyr1 = IO::load_obj(MESH_FOLDER"TUNA_p3a.obj");
	auto pyr2 = IO::load_obj(MESH_FOLDER"TUNA_p3b.obj");
	auto bool_solver = Boolean::ISolver::make();
	bool_solver->init(pyr1, pyr2);
	auto result = bool_solver->compute(Boolean::Operation::UNION);
	IO::save_obj("result_tuna_p3.obj", result);
	Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
	REQUIRE(bv_it.size() == 14);
	Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
	REQUIRE(bf_it.size() == 11);
}

TEST_CASE("apple_piece0", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"Apple_piece_0a.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"Apple_piece_0b.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_apple_piece0.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 20);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 14);
}

TEST_CASE("apple_piece1", "[Bool]")
{
  auto pyr1 = IO::load_obj(MESH_FOLDER"Apple_piece_1a.obj");
  auto pyr2 = IO::load_obj(MESH_FOLDER"Apple_piece_1b.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(pyr1, pyr2);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_apple_piece1.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  REQUIRE(bv_it.size() == 12);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 6);
}

TEST_CASE("Biplane_00", "[Bool]")
{
  auto a = IO::load_obj(MESH_FOLDER"Biplane_00_a.obj");
  auto b = IO::load_obj(MESH_FOLDER"Biplane_00_b.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(b);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] += 0.1;
    x->set_geom(pt);
  }
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(a, b);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_Biplane_00.obj", result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv1_it(result);
  REQUIRE(bv1_it.size() == 40);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == 28);
}

#if 0

TEST_CASE("Biplane", "[Bool]")
{
  auto a = IO::load_obj(MESH_FOLDER"Biplane.obj");
  auto b = IO::load_obj(MESH_FOLDER"Biplane.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(b);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] += 0.1;
    x->set_geom(pt);
  }
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(a, b);
  auto result = bool_solver->compute(Boolean::Operation::DIFFERENCE);
  IO::save_obj("result_Biplane.obj", result);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv1_it(result);
  //REQUIRE(bv1_it.size() == 14);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 10);
}
#endif

namespace {

void budda_test(const char* _number, int _expected_faces, int _expected_verts)
{
  std::string file_name = MESH_FOLDER"buddha_";
  file_name += _number;
  auto a = IO::load_obj((file_name + "a.obj").c_str());
  auto b = IO::load_obj((file_name + "b.obj").c_str());
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(a, b);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  std::string out_file = "result_buddha_";
  IO::save_obj((out_file + _number + ".obj").c_str(), result);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv1_it(result);
  REQUIRE(bv1_it.size() == _expected_verts);
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  REQUIRE(bf_it.size() == _expected_faces);
}

}//namespace

TEST_CASE("buddha_00", "[Bool]")
{
  budda_test("00", 1740, 1010);
}

TEST_CASE("buddha_01", "[Bool]")
{
  budda_test("01", 135, 111);
}

TEST_CASE("buddha_02", "[Bool]")
{
  budda_test("02", 25, 25);
}

TEST_CASE("buddha_03", "[Bool]")
{
  budda_test("03", 1272, 738);
}

TEST_CASE("buddha_04", "[Bool]")
{
  budda_test("04", 81, 57);
}

TEST_CASE("buddha_05", "[Bool]")
{
  budda_test("05", 40, 36);
}

TEST_CASE("buddha_06", "[Bool]")
{
  budda_test("06", 29, 26);
}

TEST_CASE("buddha_07", "[Bool]")
{
  budda_test("07", 31, 34);
}

TEST_CASE("buddha_08", "[Bool]")
{
  budda_test("08", 32, 27);
}

TEST_CASE("buddha_09", "[Bool]")
{
  budda_test("09", 17, 22);
}

TEST_CASE("buddha_10", "[Bool]")
{
  budda_test("10", 1803157, 913418);
}

TEST_CASE("buddha_13", "[Bool]")
{
  budda_test("13", 4, 8);
}

//TEST_CASE("buddha_11", "[Bool]") {  budda_test("11", 30, 365541); }

namespace {

void filter(Topo::Wrap<Topo::Type::BODY>& _body, 
            const Geo::Point& c, double dist)
{
  std::vector<Topo::IBase*> faces_to_remove;
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(_body);
  for (auto f : bf_it)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv_it(f);
    bool take = false;
    for (auto v : fv_it)
    {
      Geo::Point pt;
      v->geom(pt);
      if (Geo::length(pt - c) > dist)
        continue;
      take = true;
      break;
    }
    if (!take)
      //f->remove();
      faces_to_remove.push_back(f.get());
  }
  _body->remove_children(faces_to_remove);
}

Topo::Wrap<Topo::Type::BODY> budda_bools(const char* str_off)
{
  double offset = std::stof(str_off);
  auto a = IO::load_obj(MESH_FOLDER"buddha.obj");
  auto b = IO::load_obj(MESH_FOLDER"buddha.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(b);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt[2] += offset;
    x->set_geom(pt);
  }
  //filter(a);
  //IO::save_obj("debug___00.obj", a);
  //filter(b);
  //IO::save_obj("debug___01.obj", b);

  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(a, b);
  auto result = bool_solver->compute(Boolean::Operation::INTERSECTION);
  auto out_name = std::string("result_buddha_") + str_off + ".obj";
  IO::save_obj(out_name.c_str(), result);
  return result;
}

}

TEST_CASE("buddha_100_0.04", "[Bool]")
{
  auto res = budda_bools("0.04");
}

TEST_CASE("buddha_100_0.01", "[Bool]")
{
  auto res = budda_bools("0.01");
}

namespace {
void move_body(Topo::Wrap<Topo::Type::BODY> _body, const Geo::Vector3& _off)
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(_body);
  for (auto& x : bv_it)
  {
    Geo::Point pt;
    x->geom(pt);
    pt += _off;
    x->set_geom(pt);
  }
}
}

TEST_CASE("bambolina", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina.obj");
  Geo::Vector3 off0{ 0.1, 0.1, 0.1 };
  Geo::Vector3 off(off0);
  for (int i = 0; i < 3; ++i)
  {
    auto b1 = IO::load_obj(MESH_FOLDER"bambolina.obj");
    move_body(b1, off);
    off += off0;
    auto bool_solver = Boolean::ISolver::make();
    bool_solver->init(b0, b1);
    b0 = bool_solver->compute(Boolean::Operation::UNION);
    auto flnm = std::string("bambolina_part_") + 
      std::to_string(i) + ".obj";
    IO::save_obj(flnm.c_str(), b0);
  }
  IO::save_obj("result_bambolina.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_00", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_00.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_00.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  auto result = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_00.obj", result);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_01", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_00.obj");
  for (int i = 1; i < 8; ++i)
  {
    std::stringstream ss;
    ss << MESH_FOLDER << "bambolina_0" << std::to_string(i) << ".obj";
    auto b1 = IO::load_obj(ss.str().c_str());
    auto bool_solver = Boolean::ISolver::make();
    bool_solver->init(b0, b1);
    b0 = bool_solver->compute(Boolean::Operation::UNION);
    auto out_flnm = std::string("result_bambolina_01_0") + std::to_string(i) + ".obj";
    IO::save_obj(out_flnm.c_str(), b0);
  }
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_02", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_02.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_02.obj");
  const double dist = 0.03;
  Geo::Point pt_sel{ -0.18138191099083475, 0.81187284217155131, -0.13104286764790701 };
  //Geo::Point pt_sel{ 0.16645552112884549, 0.83080013530992458, -0.042750927121052280 };
  //Geo::Point pt_sel{ 0.16308298455324305, 0.72173600663393245, -0.16341504329380221 };
  //filter(b0, pt_sel, dist);
  //filter(b1, pt_sel, dist);
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::UNION);
  IO::save_obj("result_bambolina_02.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_03", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_03.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_03.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_03.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_04", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_04.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_04.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_04.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_05", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_05.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_05.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_05.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_06", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_06.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_06.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_06.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_07", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_07.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_07.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_07.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_08", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_08.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_08.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_08.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}


TEST_CASE("bambolina_09", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_09.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_09.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_09.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_10", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_10.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_10.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_10.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_11", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_11.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_11.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_11.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_13", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_13.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_13.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::SPLIT);
  IO::save_obj("result_bambolina_13.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}

TEST_CASE("bambolina_14", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_14.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_14.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::UNION);
  IO::save_obj("result_bambolina_14.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}


TEST_CASE("bambolina_15", "[Bool]")
{
  auto b0 = IO::load_obj(MESH_FOLDER"bambolina_a_15_R.obj");
  auto b1 = IO::load_obj(MESH_FOLDER"bambolina_b_15.obj");
  auto bool_solver = Boolean::ISolver::make();
  bool_solver->init(b0, b1);
  b0 = bool_solver->compute(Boolean::Operation::UNION);
  IO::save_obj("result_bambolina_15.obj", b0);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv_it(result);
  //REQUIRE(bv_it.size() == 12);
  //Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf_it(result);
  //REQUIRE(bf_it.size() == 6);
}
