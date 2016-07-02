#pragma once

#include <array>
#include <memory>

namespace Geo {

typedef std::array<double, 3> Point;
typedef std::array<Point, 2> Segment;
typedef std::array<Point, 3> Triangle;

struct IPolygonalFace
{
  virtual ~IPolygonalFace() {}
  virtual bool triangle(size_t _idx, Triangle& _tri) const = 0;
  virtual size_t triangle_number() const = 0;
  virtual Point normal() const = 0;
  template <class IteratorT>
  static std::shared_ptr<IPolygonalFace> make(const IteratorT& _beg, const IteratorT& _end)
  {
    auto poly_face = make_me();
    for (auto it = _beg; it != _end; ++it)
      poly_face->add_point(*it);
    poly_face->finalize();
    return poly_face;
  }
protected:
  virtual void add_point(const Point& _pt) = 0;
  virtual void finalize() = 0;
private:
  static std::shared_ptr<IPolygonalFace> make_me();
};


Point evaluate(const Segment& _seg, double _t);

bool closest_point(const Segment& _seg, const Point& _pt,
  Point* _clsst_pt = nullptr, double* _t = nullptr, 
  double * _dist_sq = nullptr);

/*! Finds the internal points at minimum distance between two segments.
    Returns false 
    if the two segments are parallel (according to the numeric precision) or
    if the minimum distance point between the two segment is on an end of 
    at least one the two segments.
*/
bool closest_point(const Segment& _seg_a, const Segment& _seg_b,
  Point* _clsst_pt = nullptr, double _t[2] = nullptr, double * _dist = nullptr);

/*! Finds the internal point of a triangle closest to a point, ad its distance.
The closest point is returned only if it is strictly inside the triangle.
*/
bool closest_point(const Triangle& _tri, const Point& _pt,
  Point* _clsst_pt, double * _dist_sq);

/*! Finds the internal point of a PolygonalFace closest to a point, ad its distance.
The closest point is returned only if it is strictly inside the PolygonalFace.
It is possible to miss a collision if the point is at an internal boundary of
the triangles used to create the face.
*/
bool closest_point(const IPolygonalFace& _face, const Point& _pt,
  Point* _clsst_pt = nullptr, double * _dist_sq = nullptr);

bool closest_point(const Triangle& _tri, const Segment& _seg,
  Point* _clsst_pt = nullptr, double * _t = nullptr, double * _dist_sq = nullptr);

bool closest_point(const IPolygonalFace& _face, const Segment& _seg,
  Point* _clsst_pt = nullptr, double * _t = nullptr, double * _dist_sq = nullptr);

}//namespace Geo
