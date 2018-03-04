#include "Geo/vector.hh"
#include <memory>
#include <vector>

namespace Geo
{
// Given a set of polygons, computes a triangulation.
// Polygons are supposed to be approximately on a plane
// mot intersecting and one of them must include all
// the others.
// Returns a vector of triplets accessible via the method
// triangles. The triplets are indices of points in the vector
// returned by the method polygon.
struct IPolygonTriangulation
{
  // Add a polygon. The set of added polygons must one boundary +
  // a set of islands.
  virtual void add(const std::vector<Geo::VectorD3>& _plgn) = 0;

  // A list of triplets that are indeces of points in the vector 
  // returned by method polygon.
  virtual const std::vector<std::array<size_t, 3>>& triangles() = 0;

  // The area of the triangularization.
  virtual double area() = 0;

  // The vector of points. If there was only one input polygon,
  // it is exactly it.
  virtual const std::vector<Geo::VectorD3>& polygon() = 0;

  static std::shared_ptr<IPolygonTriangulation> make();
}; // struct IPolygonTriangulation

} // namespace Geo