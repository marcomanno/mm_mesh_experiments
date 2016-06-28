#include "point_fitting.hh"
#include "C:/Users/marco/OneDrive/Documents/PROJECTS/ThirdParties/eigen-eigen-07105f7124f9/Eigen"

namespace Geo
{
struct PointFittingImpl : public PointFitting
{
  virtual void add_point(Vector3& _pt) override;
  virtual bool compute(Vector3& _centr, Vector3& _norm) override;
};

static std::shared_ptr<PointFitting> make()
{
  return std::make_shared<PointFittingImpl>();
}

}//namespace Geo