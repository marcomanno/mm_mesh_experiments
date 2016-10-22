
#pragma once

#include "vector.hh"

#include <memory>

namespace Geo {

/*! Finds the best plane for N 3d points (the pane that minimize 
    the square distance of any point from the plane.
*/
struct IPlaneFit
{
  /*!Set the expected number of points to be considered for minimum distance.
  It must not be smaller than the number of points added with add_point.
  */
  virtual void init(size_t _size) = 0;

  /*!Set a new point.
  */
  virtual void add_point(const Vector3& _pt) = 0;

  /*!Computes the best plane as the plane passing for the _center
  with the given normal. The normal is a unit vector.
  */
  virtual bool compute(Vector3& _center, Vector3& _normal) = 0;

  /*! Factory */
  static std::shared_ptr<IPlaneFit> make();
};

}//namespace Geo
