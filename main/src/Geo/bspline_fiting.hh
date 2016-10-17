#include <Geo/vector.hh>
#include <functional>

#include <vector>

namespace Geo {
namespace BsplineFItting {

template <size_t dimT>
using Function = Geo::Vector<dimT>(const double _t);

template <size_t dimT> bool solve(
  const size_t _deg, Function<dimT>* _f,
  const std::vector<double>& _knots,
  std::vector<Vector<dimT>>& _opt_ctr_pts);

}//namespace BsplineFItting
}//namespace Geo
