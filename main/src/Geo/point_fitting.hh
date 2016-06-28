#pragma once

#include "vector.hh"
#include <memory>

namespace Geo
{
struct PointFitting
{
  virtual void add_point(Vector3& _pt) = 0;
  virtual bool compute(Vector3& _centr, Vector3& _norm) = 0;
  static std::shared_ptr<PointFitting> make();
};

}//namespace Geo