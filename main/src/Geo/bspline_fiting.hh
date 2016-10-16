#include <Geo/vector.hh>
#include <functional>

#include <vector>

namespace Geo {
namespace BsplineFItting {

//template <size_t dimT>
//using Function = Vector<dimT>()(const double _t);

template <size_t dimT>
using Function = Vector<dimT>(const double _t);

template <size_t dimT> bool solve(
  const size_t _deg, const size_t _ref_lev,
  Function<dimT>* _f,
  std::vector<double>& _knots,
  std::vector<Vector<dimT>>& _opt_ctr_pts);

}//namespace BsplineFItting
}//namespace Geo
