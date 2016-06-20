#include "poly_triang.hh"
#include "geo_max_min.hh"
#include <numeric>

struct PolygonFilImpl : public PolygonFil
{
  void init(const std::vector<Geo::Vector3>& _plgn);
  const std::vector<std::array<size_t, 3>>& triangles() const
  {
    return tri_;
  }
  const std::vector<Geo::Vector3>& positions() const
  {
    return pos_;
  }

  std::vector<std::array<size_t, 3>> tri_;
  std::vector<Geo::Vector3> pos_;
private:
  bool make_triangle(std::vector<size_t>& next);
};

std::shared_ptr<PolygonFil> PolygonFil::make()
{
  return std::make_shared<PolygonFilImpl>();
}


void PolygonFilImpl::init(const std::vector<Geo::Vector3>& _plgn)
{
  pos_ = _plgn;
  tri_.clear();
  std::vector<size_t> indcs(_plgn.size());
  std::iota(indcs.begin(), indcs.end(), 0);
  while (make_triangle(indcs));
}

bool PolygonFilImpl::make_triangle(std::vector<size_t>& _indcs)
{
  if (_indcs.size() < 3)
    return false;
  Geo::Vector3 vects[2];
  auto last = _indcs.end();
  auto orig = pos_[*std::prev(last)];
  vects[0] = pos_[*std::prev(last)] - orig;

  Geo::StatisticsT<double> min_ang;

  for (const auto& ind : _indcs)
  {
    vects[1] = pos_[ind] - orig;
    min_ang.add(Geo::angle(vects[0], vects[1]));
    orig = pos_[ind];
    vects[0] = -vects[1];
  }
  auto idx = min_ang.min_idx();
  auto decrease = [&_indcs](size_t& _idx)
  {
    if (_idx > 0) --_idx;
    else _idx = _indcs.size() - 1;
    return _idx;
  };
  std::array<size_t, 3> tri;
  tri[2] = _indcs[idx];
  tri[1] = _indcs[decrease(idx)];
  auto rem_idx = idx;
  tri[0] = _indcs[decrease(idx)];
  _indcs.erase(_indcs.begin() + rem_idx);
  tri_.push_back(tri);
  return true;
}

