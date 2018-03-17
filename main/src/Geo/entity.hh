#pragma once

#include "Geo/vector.hh"

#include <array>
#include <memory>

namespace Gen {
template <class TypeT, size_t DimT> using Segment = 
  Geo::Vector<Geo::Vector<TypeT, DimT>, 2>;

template <class TypeT, size_t DimT>
auto evaluate(const Segment<TypeT, DimT>& _seg, double _t)
{
  return (1 - _t) * _seg[0] + _t * _seg[1];
}


template <class TypeT, size_t DimT, bool infT = false> 
bool closest_point(
  const Segment<TypeT, DimT>& _seg, const Geo::Vector<TypeT, DimT>& _pt,
  Geo::Vector<TypeT, DimT>* _clsst_pt = nullptr,
  TypeT* _t = nullptr,
  TypeT * _dist_sq = nullptr)
{
  auto a = _pt - _seg[0];
  auto b = _seg[1] - _seg[0];
  auto len_sq = Geo::length_square(b);
  auto len_ref_sq = 10 * std::numeric_limits<TypeT>::epsilon() *
    std::max(Geo::length_square(_seg[0]), Geo::length_square(_seg[1]));
  if (len_sq <= len_ref_sq)
    return false;
  auto t = (a * b) / len_sq;
  if constexpr (!infT)
  {
    if (t <= 0 || t >= 1)
      return false;
  }
  if (_t != nullptr) *_t = t;
  if (_clsst_pt != nullptr) *_clsst_pt = _seg[1] * t + (1 - t) * _seg[0];
  if (_dist_sq != nullptr)
    *_dist_sq = Geo::length_square(a - t * b);
  return true;
  }

template <class TypeT, size_t DimT, bool Inf1 = false, bool Inf2 = false>
bool closest_point(const Segment<TypeT, DimT>& _seg_a, const Segment<TypeT, DimT>& _seg_b,
                   Geo::Vector<TypeT, DimT>* _clsst_pt = nullptr, 
                   double _t[2] = nullptr, double * _dist = nullptr);


} // namespace Gen

namespace Geo {

typedef VectorD3 Point;
typedef std::array<Point, 2> Segment;
typedef std::array<Point, 3> Triangle;

struct IPolygonalFace
{
  virtual ~IPolygonalFace() {}
  virtual bool triangle(size_t _idx, Triangle& _tri) const = 0;
  virtual size_t triangle_number() const = 0;
  virtual Point normal() const = 0;
  static std::shared_ptr<IPolygonalFace> make();
  template <class IteratorT> void add_loop(const IteratorT& _beg, const IteratorT& _end)
  {
    const auto loop_num = make_new_loop();
    for (auto it = _beg; it != _end; ++it)
      add_point(*it, loop_num);
  }
  virtual void compute() = 0;
protected:
  virtual size_t make_new_loop() = 0;
  virtual void add_point(const Point& _pt, size_t _loop_num) = 0;
};


Point evaluate(const Segment& _seg, double _t);

inline bool 
closest_point(const Segment& _seg, const Point& _pt,
              Point* _clsst_pt = nullptr, double* _t = nullptr,
              double * _dist_sq = nullptr)
{
  return Gen::closest_point<double, 3>(_seg, _pt, _clsst_pt, _t, _dist_sq);
}

/*! Finds the internal points at minimum distance between two segments.
    Returns false 
    if the two segments are parallel (according to the numeric precision) or
    if the minimum distance point between the two segment is on an end of 
    at least one the two segments.
*/
inline bool closest_point(const Segment& _seg_a, const Segment& _seg_b,
                          Point* _clsst_pt = nullptr, double _t[2] = nullptr, double * _dist_sq = nullptr)
{
  return Gen::closest_point<double, 3>(_seg_a, _seg_b, _clsst_pt, _t, _dist_sq);
}

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
