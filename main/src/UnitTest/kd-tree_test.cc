#include "Catch/catch.hpp"
#include "Geo/kd-tree.hh"

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
    box.fatten(0.000001);
    return box;
  }
};

void print_box(const Geo::Range<3>& _box)
{
  std::cout << _box.extr_[0] << "--" << _box.extr_[1] << "\n";
}

TEST_CASE("Kd-Tree1", "[KDTREE]")
{
  std::vector<KdTreeElement> kk(123);
  Geo::KdTree<KdTreeElement> dd;
  dd.insert(kk.begin(), kk.end());
  dd.compute();
  std::function<void(size_t)> process_kt_element =
    [&dd, &process_kt_element](size_t _i)
  {
    if (_i >= dd.depth())
    {
      size_t st, en;
      if (!dd.leaf_range(_i, st, en))
        return;
      std::cout << st << " " << en << "\n";
      /*
      for (auto i = st; i != en; ++i)
      {
        std::cout << dd[i].box().extr_[0] << "--" << dd[i].box().extr_[1] << "\n";
      }*/
    }
    else
    {
      for (auto j : { 0,1 })
      {
        std::cout << "Node (" << _i << "," << j << "):";
        print_box(dd.box(_i, j));
        process_kt_element(dd.child(_i, j));
      }
    }
  };
  process_kt_element(0);
}
