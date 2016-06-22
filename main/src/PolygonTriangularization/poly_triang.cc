#include "poly_triang.hh"
#include "statistics.hh"
#include <numeric>

struct PolygonFilImpl : public PolygonFil
{
  void init(const std::vector<Geo::Vector3>& _plgn);
  const std::vector<std::array<size_t, 3>>& triangles() const
  {
    return tris_;
  }
  const std::vector<Geo::Vector3>& positions() const
  {
    return pos_;
  }

  std::vector<std::array<size_t, 3>> tris_;
  std::vector<Geo::Vector3> pos_;
  double area_ = 0;
private:
  bool add_triangle(std::vector<size_t>& _indcs);
  double compute_area(
    const std::vector<std::array<size_t, 3>>& _tris) const;
  struct Solution
  {
    std::vector<std::array<size_t, 3>> tris_;
    double area_ = 0;
    std::vector<bool> concav_;
  };
};

std::shared_ptr<PolygonFil> PolygonFil::make()
{
  return std::make_shared<PolygonFilImpl>();
}


void PolygonFilImpl::init(const std::vector<Geo::Vector3>& _plgn)
{
  pos_ = _plgn;
  tris_.clear();
  std::vector<size_t> indcs(_plgn.size());
  std::iota(indcs.begin(), indcs.end(), 0);
  while (add_triangle(indcs));
}

bool PolygonFilImpl::add_triangle(std::vector<size_t>& _indcs)
{
  if (_indcs.size() < 3)
    return false;
  Geo::Vector3 vects[2];
  auto last = std::prev(_indcs.end());
  auto orig = pos_[*last];
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
    if (_idx == 0) _idx = _indcs.size();
    return --_idx;
  };
  std::array<size_t, 3> tri;
  tri[2] = _indcs[idx];
  tri[1] = _indcs[decrease(idx)];
  auto rem_idx = idx;
  tri[0] = _indcs[decrease(idx)];
  _indcs.erase(_indcs.begin() + rem_idx);
  if (min_ang.min() > 0)
    tris_.push_back(tri);
  return true;
}

double PolygonFilImpl::compute_area(
  const std::vector<std::array<size_t, 3>>& _tris) const
{
  double area = 0;
  for (const auto& tri : _tris)
  {
    const auto v0 = pos_[tri[1]] - pos_[tri[0]];
    const auto v1 = pos_[tri[2]] - pos_[tri[0]];
    area += Geo::length(v0 % v1);
  }
  return area / 2;
}
