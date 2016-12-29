#include "kd-tree.hh"

struct KdTreeElement
{
  static const size_t DIM = 3;
  Geo::Vector3 pt_ =
  {
    double(std::rand()) / RAND_MAX,
    double(std::rand()) / RAND_MAX,
    double(std::rand()) / RAND_MAX
  };
  Geo::Range<3> box_;
  const Geo::Vector3& point() const { return pt_; }
  const Geo::Range<3> box() const 
  {
    Geo::Range<3> box;
    box.extr_[0] = box.extr_[1] = pt_;
    box.fatten(0.001);
    return box;
  }
};

struct F
{
  F()
  {
    std::vector<KdTreeElement> kk(100);
    Geo::KdTree<KdTreeElement> dd;
    dd.insert(kk.begin(), kk.end());
    dd.compute();
  }
} f;
