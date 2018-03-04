
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
  virtual void add_point(const VectorD3& _pt) = 0;

  /*!Computes the best plane as the plane passing for the _center
  with the given normal. The normal is a unit vector.
  */
  virtual bool compute(
    VectorD3& _center,
    VectorD3& _normal,
    const bool _orient = false) = 0;

  /*! Factory */
  static std::shared_ptr<IPlaneFit> make();
};

/*! Retun the normal of the best plane in a set of points.
    The normal is such that looking at the polygon from the
    normal direction it runs in counterclockwise direction.
*/
template <class VertexIteratorT>
Geo::VectorD3 vertex_polygon_normal(VertexIteratorT _beg, VertexIteratorT _end)
{
  auto pl_fit = Geo::IPlaneFit::make();
  pl_fit->init(_end - _beg);
  for (auto vert_it = _beg; vert_it != _end; ++vert_it)
  {
    Geo::VectorD3 pt;
    (*vert_it)->geom(pt);
    pl_fit->add_point(pt);
  }
  Geo::VectorD3 c, n;
  pl_fit->compute(c, n, true);
  return n;
}

template <class PointIteratorT>
Geo::VectorD3 point_polygon_normal(
  PointIteratorT _beg, PointIteratorT _end, 
  Geo::VectorD3* _centr = nullptr)
{
  auto pl_fit = Geo::IPlaneFit::make();
  pl_fit->init(_end - _beg);
  for (auto pt_it = _beg; pt_it != _end; ++pt_it)
    pl_fit->add_point(*pt_it);
  Geo::VectorD3 c, n;
  pl_fit->compute(c, n, true);
  if (_centr != nullptr)
    *_centr = c;
  return n;
}

}//namespace Geo
