#include "catch/catch.hpp"

#include <Geo/bspline_fiting.hh>
#include <Geo/evalnurbs.hh>


#include <fstream>
namespace {

void analyse(const std::vector<double>& _knots,
  const Geo::IBsplineFitting<2>::IFunction& _func)
{
  std::ofstream plot("table.txt");

  std::vector<std::shared_ptr<Geo::IBsplineFitting<2>>> fitters(4);
  std::vector<Geo::Nub<Geo::VectorD<2>, double>> ev_nubs(fitters.size());
  for (auto& bsp_fit : fitters)
  {
    bsp_fit = Geo::IBsplineFitting<2>::make();
    bsp_fit->init(2, _knots, _func);
  }
  plot << "Default Internal ParCorrection Precise \n";
  fitters[1]->set_favour_boundaries(false);
  fitters[2]->set_parameter_correction_iterations(7);
  fitters[3]->set_samples_per_interval(256);
  std::vector<std::vector<Geo::VectorD<2>>> ctrl_pts_store;
  for (size_t i = 0; i < fitters.size(); ++i)
  {
    fitters[i]->compute();
    ctrl_pts_store.push_back(fitters[i]->X());
    ev_nubs[i].init(ctrl_pts_store[i], _knots);
  }
  for (double x = 0; x <= 1.; x += 1. / 256)
  {
    const double t = _knots.back() * x + _knots.front() * (1 - x);
    Geo::VectorD<2> pt_bspl, pt_crv;
    for (auto& ev_nub : ev_nubs)
    {
      ev_nub.eval(t, &pt_bspl, &pt_bspl + 1);
      double dist;
      if (x == 0 || x == 1)
        pt_crv = _func.evaluate(t);
      else
        pt_crv = _func.closest_point(pt_bspl, t);
      dist = Geo::length(pt_bspl - pt_crv);
      plot << dist << " ";
    }
    plot << std::endl;
  }
}

}//namespace



TEST_CASE("arc fit", "[BSPLFIT]")
{
  std::vector<double> knots = { 0, 0, 1. / 64, 1. / 32, 0.0625, 0.125, 0.25, 0.5, 1, 1 };
  struct Function : public Geo::IBsplineFitting<2>::IFunction
  {
    const double coe_ = M_PI * 3 / 2;
    virtual Geo::VectorD<2> evaluate(const double _t) const
    {
      return Geo::VectorD<2>{sin(_t * coe_), cos(_t * coe_)};
    }
    virtual Geo::VectorD<2> closest_point(
      const Geo::VectorD<2>& _pt, const double) const
    {
      return _pt / Geo::length(_pt);
    }
  };
  analyse(knots, Function());
}


TEST_CASE("poly fit", "[BSPLFIT]")
{
  std::vector<double> knots = { 0, 0, 1. / 64, 1. / 32, 0.0625, 0.125, 0.25, 0.5, 1, 1 };
  struct Function : public Geo::IBsplineFitting<2>::IFunction
  {
    const double one_third = 1./ 3, two_third = 2. / 3;
    virtual Geo::VectorD<2> evaluate(const double _t) const
    {
      if (_t < one_third)
        return Geo::VectorD<2>{ _t * 3, 1 };
      if (_t > two_third)
        return Geo::VectorD<2>{ 1. - _t, -1. };
      return Geo::VectorD<2>{ 1., 3. - _t * 6. };
    }
    virtual Geo::VectorD<2> closest_point(
      const Geo::VectorD<2>& _pt, const double _t) const
    {
      if (_t < one_third)
      {
        double x = std::max(std::min(_pt[0], 1.), 0.);
        return Geo::VectorD<2>{ x, 1. };
      }
      if (_t > two_third)
      {
        double x = std::max(std::min(_pt[0], 1.), 0.);
        return Geo::VectorD<2>{ x, -1. };
      }
      double y = std::max(std::min(_pt[1], 1.), -1.);
      return Geo::VectorD<2>{ 1., y };
    }
  };
  analyse(knots, Function());
}
