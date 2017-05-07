#include "catch/catch.hpp"

#include <Import/import.hh>
#include <PolygonTriangularization/poly_triang.hh>

#include <fstream>

static void write_obj(const char* _flnm,
  const std::vector<Geo::Vector3>& _plgn,
  const std::vector<std::array<size_t, 3>>& _tris)
{
  std::ofstream ff(std::string(_flnm) + ".obj");
  for (const auto& v : _plgn) { ff << "v" << v << "\n"; }
  for (auto f : _tris)
  {
    for (auto& fi : f)
      fi += 1;
    ff << "f" << f << "\n";
  }
}

static void write_obj(const char* _flnm,
  const std::vector<Geo::Vector3>& _plgn)
  {
  IO::save_obj(_flnm, _plgn);
  }

#define WRITE_OBJ(A, B, C) write_obj("result_" A, B, C)

#undef TEST_NAME
#define TEST_NAME "poly_1"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0, 0, 0 });
  plgn.push_back({ 0, 1, 0 });
  plgn.push_back({ 1, 1, 0 });
  plgn.push_back({ 1, 0, 0 });
  plgn.push_back({ 0.5, 0.5, 0 });
  ptg->add(plgn);
  auto& tris = ptg->triangles();
  REQUIRE(tris.size() == 3);
  REQUIRE(ptg->area() == 0.75);
  write_obj(TEST_NAME, plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "poly_2"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 50,   0, 0 });
  plgn.push_back({ 25,  45, 0 });
  plgn.push_back({ -25,  45, 0 });
  plgn.push_back({ 0,  25, 0 });
  plgn.push_back({ -50,   0, 0 });
  plgn.push_back({ -25, -45, 0 });
  plgn.push_back({ 25, -45, 0 });
  plgn.push_back({ 0, -25, 0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(ptg->area() == 5125);
  WRITE_OBJ(TEST_NAME, plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "poly_3"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0,    0,  0 });
  plgn.push_back({ 1,    0,  0 });
  plgn.push_back({ 1,    1, 0 });
  plgn.push_back({ 1.25, 0, 0 });
  plgn.push_back({ 2,    0, 0 });
  plgn.push_back({ 2,    2, 0 });
  plgn.push_back({ 0,    2, 0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, plgn, tris);
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(ptg->area() == 3.875);
}

#undef TEST_NAME
#define TEST_NAME "poly_4"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0,   1.5, 0 });
  plgn.push_back({ 1,   0,   0 });
  plgn.push_back({ 1,   0.5, 0 });
  plgn.push_back({ 1.5, 0,   0 });
  plgn.push_back({ 2,   0,   0 });
  plgn.push_back({ 2,   2,   0 });
  plgn.push_back({ 0,   2,   0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(ptg->area() == 3.125);
  WRITE_OBJ(TEST_NAME, plgn, tris);
}

#undef TEST_NAME
#define TEST_NAME "poly_5"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0,   0,   0 });
  plgn.push_back({ 2,   0,   0 });
  plgn.push_back({ 2,   1,   0 });
  plgn.push_back({ 1,   1,   0 });
  plgn.push_back({ 2,   2,   0 });
  plgn.push_back({ 3,   2,   0 });
  plgn.push_back({ 3,   0,   0 });
  plgn.push_back({ 4,   0,   0 });
  plgn.push_back({ 4,   4,   0 });
  plgn.push_back({ 0,   4,   0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, plgn, tris);
  REQUIRE(ptg->area() == 13.5);
  REQUIRE(tris.size() == plgn.size() - 2);
}


#undef TEST_NAME
#define TEST_NAME "poly_6"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn;

  plgn.push_back({ 0, 0, 0 });
  plgn.push_back({ 1, 0, 0 });
  plgn.push_back({ 1, 1, 0 });
  plgn.push_back({ 0, 2, 0 });
  plgn.push_back({ -1, 2, 0 });
  plgn.push_back({ -2, 0, 0 });
  plgn.push_back({ -2,-2, 0 });
  plgn.push_back({ 0,-4, 0 });
  plgn.push_back({ 0,-3, 0 });
  plgn.push_back({ -1,-2, 0 });
  plgn.push_back({ -1,-1, 0 });
  plgn.push_back({ -1.5, 0, 0 });
  plgn.push_back({ -1, 0, 0 });
  plgn.push_back({ 0, 1, 0 });

  ptg->add(plgn);
  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, plgn, tris);
  REQUIRE(ptg->area() == 7.25);
  REQUIRE(tris.size() == plgn.size() - 2);
}


#undef TEST_NAME
#define TEST_NAME "poly_7"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();

  std::vector<Geo::Vector3> plgn;
  plgn.push_back({ 0, 0, 0 });
  plgn.push_back({ 3, 0, 0 });
  plgn.push_back({ 3, 3, 0 });
  plgn.push_back({ 0, 3, 0 });
  ptg->add(plgn);

  plgn.clear();
  plgn.push_back({ 1, 2, 0 });
  plgn.push_back({ 2, 2, 0 });
  plgn.push_back({ 2, 1, 0 });
  plgn.push_back({ 1, 1, 0 });
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(ptg->area() == 8);
  REQUIRE(tris.size() == 8);
}

#undef TEST_NAME
#define TEST_NAME "poly_8"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  {
    std::vector<Geo::Vector3> plgn =
    {
      { 1, 0, 0 },
      { 2, 1, 0 },
      { 1, 2, 0 },
      { 2, 3, 0 },
      { 1, 4, 0 },
      { 2, 5, 0 },
      { 1, 6, 0 },
      {-1, 6, 0 },
      {-2, 5, 0 },
      {-1, 4, 0 },
      {-2, 3, 0 },
      {-1, 2, 0 },
      {-2, 1, 0 },
      {-1, 0, 0 }
    };
    ptg->add(plgn);
  }
  {
    std::vector<Geo::Vector3> plgn =
    {
      { -0.5,  1.5, 0 },
      {  0.5,  1.5, 0 },
      { 0.5,  0.5, 0 },
      { -0.5,  0.5, 0 }
    };
    ptg->add(plgn);
  }
  {
    std::vector<Geo::Vector3> plgn =
    {
      { -0.5,  4.5, 0 },
      {  0.5,  4.5, 0 },
      {  0.5,  3.5, 0 },
      { -0.5,  3.5, 0 }
    };
    ptg->add(plgn);
  }

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 24);
  REQUIRE(ptg->area() == 16);
}

#undef TEST_NAME
#define TEST_NAME "poly_9"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  {
    std::vector<Geo::Vector3> plgn =
    {
      { 0, 0, 0 },
      { 1, 0, 0 },
      { 2, 0, 0 },
      { 2, 2, 0 },
      { 0, 2, 0 },
      { 0, 1, 0 },
    };
    ptg->add(plgn);
  }

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 4);
  REQUIRE(ptg->area() == 4);
}

#undef TEST_NAME
#define TEST_NAME "poly_10"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn =
  {
    { 38.726774787455334, 0.56712230225691052, -0.144475349131792 },
    { 38.827700000000000, 0.57999999999999996, -0.765400000000000 },
    { 38.631000000000000, 0.58699999999999997, -0.927499999999998 },
    { 38.533099999999997, 0.57110000000000005, -0.161700000000000 },
    { 38.718219076363624, 0.56676148139743932, -0.144170977869948 },
    { 38.726599999999998, 0.56710000000000005, -0.143400000000000 }
  };
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 4);
  REQUIRE(ptg->area() > 0.14442415);
  REQUIRE(ptg->area() < 0.144431);
}

#undef TEST_NAME
#define TEST_NAME "poly_11"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn =
  {
    { 0.057061814887888161, -0.034494051724338268, 0.12295738396624364 },
    { 0.056968958145399401, -0.032443605256421870, 0.11787198073397483 },
    { 0.056963278374730766, -0.032395499739924227, 0.11766664515156007 },
    { 0.056954222265661256, -0.032442336173918618, 0.11750818045197592 },
    { 0.056948522027293433, -0.032342100976387088, 0.11723105716616827 },
    { 0.056653562608728336, -0.029521696175577341, 0.10612705148509588 },
    { 0.057557795199999998, -0.013568500989999999, 0.10652909130000002 }
  };
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
}

#undef TEST_NAME
#define TEST_NAME "poly_12"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn =
  {
    {-0.056809722145775982, -0.044396670069383221, -0.069267102372259484},
    { -0.056563801126929512,-0.046557666128034858,-0.07240728427777951 },
    { -0.056446485880243605,-0.047772458231279388,-0.073649348639929418 },
    { -0.056542635962607453,-0.045819371882530158,-0.073963898526133137 },
    { -0.056582128876468324,-0.045274798257252259,-0.073734527988125298 },
    { -0.056605309728997355,-0.045134155838012224,-0.073350772719993296 },
    { -0.056809722145775982,-0.044396670069383221,-0.069267102372259484 },
    { -0.057557795199999998,-0.013568500989999999,-0.093470908699999994 },
    { -0.054066501560000001,-0.075164698059999993,-0.095023207370000001 },
    { -0.056255899370000001,-0.068322397770000001,-0.050484295929999999 }
  };
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 8);
  REQUIRE(ptg->area() > 0.001364);
  REQUIRE(ptg->area() < 0.001365);
}

#undef TEST_NAME
#define TEST_NAME "poly_13"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn =
  {
    {-0.015398045507488713,0.13453929220918848,0.010226638621282969 },
    {-0.015397853867103028,0.13453474101739316,0.010227500420000001 },
    {-0.015397852522544213,0.13453471505698025,0.010227505640857022 },
    {-0.01539350022,0.1344538481535526,0.01024396764032208 }
  };
  write_obj("poly_13_in.obj", plgn);
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 2);
  REQUIRE(ptg->area() == Approx(0.001364).epsilon(0.01));
}

#undef TEST_NAME
#define TEST_NAME "poly_14"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn =
  {
  {0.032126756174994733, 0.22023025214841976, -0.0084818488195413204 },
  {0.03214375757415814 , 0.22028773621364428, -0.0085069482357158829 },
  {0.032154008277420341, 0.22032499310000001, -0.0085214760041893624 },
  {0.032154610996427808, 0.22032711228356178, -0.0085223468562670626 },
  {0.032154679062226056, 0.22032711528171794, -0.0085225002839999994 },
  {0.032174613431750654, 0.22039310946049628, -0.0085522575462263634 },
  //{0.032187281925439411, 0.22043469643157293, -0.0085712508628891654 },
  {0.032106500119999999, 0.22032499310000001, -0.0084138996935174196 } };
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 5);
  REQUIRE(ptg->area() == Approx(9.6755109902549154e-09).epsilon(1.e-12));
}

#undef TEST_NAME
#define TEST_NAME "poly_15"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn =
  {
  //{0.032126756174994733,0.22023025214841976, -0.0084818488195413204 },
  {0.03214375757415814,0.22028773621364428, -0.0085069482357158829 },
  {0.032154008277420341,0.22032499310000001, -0.0085214760041893624 },
  {0.032154610996427808,0.22032711228356178, -0.0085223468562670626 },
  {0.032154679062226056,0.22032711528171794, -0.0085225002839999994 },
  {0.032174613431750654,0.22039310946049628, -0.0085522575462263634 },
  //{0.032187281925439411,0.22043469643157293, -0.0085712508628891654 },
  {0.032106500119999999,0.22032499310000001, -0.0084138996935174196 }
  };
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 4);
  REQUIRE(ptg->area() == Approx(9.6755109902549154e-09).epsilon(1.e-8));
}

#undef TEST_NAME
#define TEST_NAME "poly_16"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
	auto ptg = IPolygonTriangulation::make();
	std::vector<Geo::Vector3> plgn =
	{
	{-0.012722799550000000, -0.0082849198949999994, -0.50010401010000005},
	{-0.0069300997999999997, -0.0043926499779999998, -0.53319698569999996},
	{-0.00012178199540000001, -0.019580800089999999, -0.50741499660000000}
	};
	ptg->add(plgn);

	std::vector<Geo::Vector3> plgn1 =
	{
	{ -0.010208646057153086, -0.010133437957891653, -0.50288889236281020 },
	{ -0.0091940914529471101, -0.011311432592683218, -0.50259874740507549 },
	{ -0.0093988336954648928, -0.010409111948058859, -0.50483229502363836 },
	{ -0.010095557860560725, -0.010014707370553207, -0.50367483650310729 },
	{ -0.010149038214034707, -0.0099844209959898469, -0.50358602954336007 },
	{ -0.010394080520089333, -0.0098457064449308657, -0.50317894391468521 },
	{ -0.010393385786015922, -0.0098519849292692495, -0.50316083778610077 },
	{ -0.010386069840100584, -0.0099182022432831966, -0.50296983842728804 },
	{ -0.010384896355011827, -0.0099287908753552135, -0.50293930892686589 },
	{ -0.010382688680357532, -0.0099313627844512427, -0.50293864946369604 }
	};
	ptg->add(plgn1);
	auto& tris = ptg->triangles();
	WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
	REQUIRE(tris.size() == 13);
	REQUIRE(ptg->area() == Approx(0.0002786241).epsilon(1.e-8));
}

#undef TEST_NAME
#define TEST_NAME "poly_17"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  auto ptg = IPolygonTriangulation::make();
  std::vector<Geo::Vector3> plgn =
  {
    { -0.12209898230000001, 0.81369900699999997, -0.20586098729999999 },
    { -0.10477899760000001, 0.7958909273, -0.2115119845 },
    { -0.10477888736169111, 0.79589290160722759, -0.21151221689864735 },
    { -0.10477883876920735, 0.79589373104768923, -0.2115123049028827 },
    { -0.1038990021, 0.81104397770000003, -0.21314898130000001 } };
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 3);
  REQUIRE(ptg->area() == Approx(0.0001508917).epsilon(1.e-8));
}

#undef TEST_NAME
#define TEST_NAME "poly_18"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  std::vector<Geo::Vector3> plgn = {
  {-0.098906010389999993, 0.76550602909999999, -0.010635999029999999},
  {-0.085294011660814045, 0.77377302735461284, -0.0012610171776285871},
  {-0.085066655823752502, 0.77130537275572841, -0.002411906405151318},
  {-0.085066038225135698, 0.77129866920243884, -0.002415032881060594},
  { -0.084091998639999999, 0.76072597500000005, -0.0073459967970000003 }};

  auto ptg = IPolygonTriangulation::make();
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == 3);
  REQUIRE(ptg->area() == Approx(0.0001109154).epsilon(1.e-8));
}

#undef TEST_NAME
#define TEST_NAME "poly_19"
TEST_CASE(TEST_NAME, "[PolyTriang]")
{
  std::vector<Geo::Vector3> plgn = {
  {-0.18554915619976867, 0.83334685546204523, -0.10658609029017733 },
  {-0.18671896450840569, 0.82393615366562467, -0.090585316377653352 },
  //{-0.18671898540000001, 0.82393598560000003, -0.090585030620000007 },
  {-0.18671899388682417, 0.82393595301191547, -0.090585374261265866 },
  {-0.18711437859974214, 0.82241773722249367, -0.10659495576081568 },
  {-0.18713198890798496, 0.82235007017989181, -0.10730799508728073 },
  {-0.18713200090000001, 0.8223500252       , -0.10730897640000001 },
  {-0.18554900590000001, 0.83334797620000001, -0.1065879986 }
  };
  write_obj("poly_19_in.obj", plgn);
  auto ptg = IPolygonTriangulation::make();
  ptg->add(plgn);

  auto& tris = ptg->triangles();
  WRITE_OBJ(TEST_NAME, ptg->polygon(), tris);
  REQUIRE(tris.size() == plgn.size() - 2);
  REQUIRE(ptg->area() == Approx(0.0000923309).epsilon(1.e-8));
}
