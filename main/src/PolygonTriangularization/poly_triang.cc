#include "poly_triang.hh"

struct PolygonFilImpl : public PolygonFil
{
  void init(const std::vector<Geo::Vector3>& _plgn);
  const std::vevtor<std::array<size_t, 3>>& triangles() const
  {
    return tri_;
  }
  const std::vevtor<Geo::Vector3>& positions() const
  {
    return pos_;
  }

  std::vevtor<std::array<size_t, 3>> tri_;
  std::vevtor<Geo::Vector3> pos_;
};

std::shared_ptr<PolygonFil> PolygonFil::make()
{
  return std::make_shared<PolygonFilImpl>();
}


void PolygonFil::init(const std::vector<Geo::Vector3>& _plgn)
{
  res_.reset(new Result(_plgn));
}



size_t PolygonFil::triangle_number() const
{
  compute(res_);
}
  bool triangle_indices(std::array<size_t, 3>& _trngl) const;
  bool vertex_position(const size_t _idx, Geo::Vector3&);

  std::vector<Geo::Vector3> plgn_;
  struct Result;
  Result* res_;
};//class PolygonFill
