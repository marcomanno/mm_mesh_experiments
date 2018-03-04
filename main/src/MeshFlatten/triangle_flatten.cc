#include "triangle_flatten.hh"

#include <vector>

namespace
{

}

namespace MeshFlatt
{

struct MF : public IMF
{
  virtual void add_point(const Geo::VectorD3& _pt) override { pts_.push_back(_pt); }
  virtual void add_facet(std::array<int, 3>& _facet) override { facets_.push_back(_facet); }
  virtual void compute() override
  {

  }
  virtual const Geo::VectorD3& get_point(int _i) override { return pts_[_i]; }

  std::vector<Geo::VectorD3> pts_;
  std::vector<std::array<int, 3>> facets_;

private:
  void triangle_contrib(size_t _i);
};


std::shared_ptr<IMF> IMF::make()
{
  return std::make_shared<MF>();  
}


void MF::triangle_contrib(size_t _i)
{
  auto& facet = facets_[_i];
  auto v1 = pts_[facet[1]] - pts_[facet[0]];
  auto v2 = pts_[facet[2]] - pts_[facet[0]];
  auto norm = v1 % v2;
  Geo::VectorD2 vp1 = { Geo::length(v1), 0};
  auto prj_on_v1 = (v2 * v1) / Geo::length(v1);
  Geo::VectorD2 vp2 = { prj_on_v1, Geo::length((v2 - prj_on_v1 * v1))};
  double det = vp1 % vp2;
  double A[2][2] = {
    { vp2[1] / det,  -vp1[1] / det },
    { -vp2[0] / det,  vp1[0] / det }
  };
  auto mixed = A[0][0] * A[0][1];
  double W00[2][2] = { 
    {Geo::sq(A[0][0]), mixed },
    { mixed, Geo::sq(A[0][1]) }
  };
  mixed = A[1][0] * A[1][1];
  double W11[2][2] = {
    { Geo::sq(A[1][0]), mixed },
    { mixed, Geo::sq(A[1][1]) }
  };
  mixed = A[1][0] * A[0][1] + A[0][0] * A[1][1];
  double W01[2][2] = {
    { 2 * A[0][0]* A[1][0], mixed },
    { mixed, 2 * A[1][1] * A[0][1] }
  };
#if 0
  double w00, w01, w11;
  auto bsquare = Geo::sq((W00[0][0] + W00[1][1]) * w00 + (W11[0][0] + W11[1][1]) * w11 + (W01[0][0] + W01[1][1]) * w01 - 2);
  auto cc = 
    (W00[0][0] * w00 + W01[0][0] * w01 + W11[0][0] * w11) * (W00[1][1] * w00 + W01[1][1] * w01 + W11[1][1] * w11) -
    Geo::sq(W00[0][1] * w00 + W01[0][1] * w01 + W11[0][1] * w11);
  bsquare; cc;
#endif
}


} // namespace MeshFlatt
