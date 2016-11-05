//
// This simple example shows how to do basic rendering and pipeline
// creation using C++.
//

#include "open_file.hh"

#include "vtkCylinderSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCamera.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include "vtkVertex.h"

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkPlot.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>

#include "Topology/iterator.hh"
#include "Import/import.hh"
#include "Boolean/boolean.hh"
#include "Geo/bspline_fiting.hh"
#include "Geo/evalnurbs.hh"


#include <map>
#include <iostream>

template<size_t idxT> void example_function();

typedef void(*ExamplePtr)();

static std::map<size_t, ExamplePtr>& example_table()
{
  static std::map<size_t, ExamplePtr> exmpls_;
  return exmpls_;
}

template <size_t nmbrT> struct Examples
{
  Examples()
  {
    example_table()[nmbrT] = example_function<nmbrT>;
  }
};

#define EXAMPLE(nmbr)     \
Examples<nmbr> ex_##nmbr; \
template<> void example_function<nmbr>()

int main()
{
  std::cout << "Select test:" << std::endl;
  for (const auto& test : example_table())
  {
    std::cout << test.first << std::endl;
  }
  size_t sel;
  std::cin >> sel;
  example_table()[sel]();
  return 0;
}

namespace {

template <class T> struct deleter
{
  void operator()(T* _vpdm) { _vpdm->Delete(); }
};
template<typename T> using VtkUniquePtr = std::unique_ptr<T, deleter<T>>;

void render_actors(std::vector<vtkPolyData*>& _ply_dats, Geo::Vector3* _clrs = nullptr)
{
  // Create the graphics structure. The renderer renders into the
  // render window. The render window interactor captures mouse events
  // and will perform appropriate camera or actor manipulation
  // depending on the nature of the events.
  //
  vtkRenderer *renderer = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer(renderer);
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  //// Setup the text and add it to the renderer
  //vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
  //textActor->SetInput("Hello world");
  //textActor->SetPosition2(10, 40);
  //textActor->GetTextProperty()->SetFontSize(24);
  //textActor->GetTextProperty()->SetColor(1.0, 0.0, 0.0);
  //renderer->AddActor2D(textActor);

  std::vector<std::tuple<VtkUniquePtr<vtkPolyDataMapper>, VtkUniquePtr<vtkActor>>>
    vtk_del;
  for (auto ply_dat : _ply_dats)
  {
    // The mapper is responsible for pushing the geometry into the graphics
    // library. It may also do color mapping, if scalars or other attributes
    // are defined.
    //
    vtkPolyDataMapper* actor_map = vtkPolyDataMapper::New();
    actor_map->SetInputData(ply_dat);
    // The actor is a grouping mechanism: besides the geometry (mapper), it
    // also has a property, transformation matrix, and/or texture map.
    // Here we set its color and rotate it -22.5 degrees.
    vtkActor* actor = vtkActor::New();
    vtk_del.emplace_back(actor_map, actor);
    actor->SetMapper(actor_map);
    if (_clrs != nullptr)
      actor->GetProperty()->SetColor((_clrs++)->data());
    else
      actor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
    //actor->RotateX(30.0);
    //actor->RotateY(-45.0);

    actor->GetProperty()->SetRepresentationToWireframe();
    actor->GetProperty()->SetPointSize(5);

    // Add the actors to the renderer, set the background and size
    //
    renderer->AddActor(actor);
    }

  renderer->SetBackground(0.1, 0.2, 0.4);
  renWin->SetSize(800, 800);

  // We'll zoom in a little by accessing the camera and invoking a "Zoom"
  // method on it.
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Zoom(1.5);
  renderer->GetActiveCamera()->SetParallelProjection(1);
  renWin->Render();

  // This starts the event loop and as a side effect causes an initial render.
  iren->Start();

  // Exiting from here, we have to delete all the instances that
  // have been created.
  renderer->Delete();
  renWin->Delete();
  iren->Delete();
}

vtkPolyData* make_tessellation(Topo::Wrap<Topo::Type::BODY> _body)
{
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> vert_it(_body);
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> all_verts;
  for (size_t i = 0; i < vert_it.size(); ++i)
    all_verts.push_back(vert_it.get(i));
  std::sort(all_verts.begin(), all_verts.end());
  all_verts.erase(std::unique(all_verts.begin(), all_verts.end()), all_verts.end());

  vtkPolyData* poly_dat = vtkPolyData::New();

  auto newPoints = vtkPoints::New();
  newPoints->Allocate(all_verts.size());
  for (auto v : all_verts)
  {
    Geo::Point pt;
    v->geom(pt);
    newPoints->InsertNextPoint(pt[0], pt[1], pt[2]);
  }
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
  for (int id = 0; id < all_verts.size(); ++id)
  {
    vtkVertex* vertex = vtkSmartPointer<vtkVertex>::New();
    vertex->GetPointIds()->SetId(0, id);
    vertices->InsertNextCell(vertex);
  }
  poly_dat->SetPoints(newPoints);
  poly_dat->SetVerts(vertices);
  newPoints->Delete();

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> face_it(_body);
  auto newPolys = vtkCellArray::New();
  newPolys->Allocate(face_it.size());
  for (size_t i = 0; i < face_it.size(); ++i)
  {
    std::vector<vtkIdType> pts;
    auto f = face_it.get(i);
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv(f);
    for (size_t j = 0; j < fv.size(); ++j)
    {
      auto v = fv.get(j);
      auto it =
        std::lower_bound(all_verts.begin(), all_verts.end(), v);
      if (it != all_verts.end() && *it == v)
      {
        pts.push_back(it - all_verts.begin());
      }
    }
    newPolys->InsertNextCell(pts.size(), pts.data());
  }
  poly_dat->SetPolys(newPolys);
  return poly_dat;
}

}

EXAMPLE(0)
{
  std::vector<vtkPolyData*> poly_dats;
  auto body0 = IO::load_obj(open_file().c_str());
  //  "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/result.obj");
  poly_dats.push_back(make_tessellation(body0));
 
  auto body1 = IO::load_obj(open_file().c_str());
  poly_dats.push_back(make_tessellation(body1));
  Geo::Vector3 cols[2] = { {1, 0, 0}, {0,0,1} };

  render_actors(poly_dats, cols);
}

EXAMPLE(1)
{
  // This creates a polygonal cylinder model with eight circumferential facets.
  //
  vtkPolyData *poly_dat = vtkPolyData::New();
  auto newPoints = vtkPoints::New();
  newPoints->Allocate(8);
  for (int idx = 0; idx < 8; ++idx)
  {
    double vv[2] = { -1, 1 };
    newPoints->InsertNextPoint(
      vv[(idx & 4) > 0], vv[(idx & 2) > 0], vv[(idx & 1) > 0]);
  }
  poly_dat->SetPoints(newPoints);
  newPoints->Delete();

  //output->GetPointData()->SetNormals(newNormals);
  //newNormals->Delete();

  //output->GetPointData()->SetTCoords(newTCoords);
  //newTCoords->Delete();

  auto newPolys = vtkCellArray::New();
  newPolys->Allocate(6);
  {
    vtkIdType pts[4] = { 0, 1, 3, 2 };
    newPolys->InsertNextCell(4, pts);
  }
  {
    vtkIdType pts[4] = { 4, 5, 7, 6 };
    newPolys->InsertNextCell(4, pts);
  }
  {
    vtkIdType pts[4] = { 0, 1, 5, 4 };
    newPolys->InsertNextCell(4, pts);
  }
  {
    vtkIdType pts[4] = { 2, 3, 7, 6 };
    newPolys->InsertNextCell(4, pts);
  }
  {
    vtkIdType pts[4] = { 0, 2, 6, 4 };
    newPolys->InsertNextCell(4, pts);
  }
  {
    vtkIdType pts[4] = { 1, 3, 7, 5 };
    newPolys->InsertNextCell(4, pts);
  }

  poly_dat->SetPolys(newPolys);
  newPolys->Delete();
  std::vector<vtkPolyData*> poly_dats{ poly_dat };
  render_actors(poly_dats);
}

EXAMPLE(2)
{
  auto body = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/pyramid.OBJ");
  vtkPolyData* poly_dat = make_tessellation(body);
  std::vector<vtkPolyData*> poly_dats{ poly_dat };
  render_actors(poly_dats);
}

EXAMPLE(3)
{
  auto body0 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA.OBJ");
  auto body1 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA.OBJ");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> vert_it(body1);
  const Geo::Vector3 oofs{ .1, .1, .1 };
  for (size_t i = 0; i < vert_it.size(); ++i)
  {
    Geo::Point pt;
    vert_it.get(i)->geom(pt);
    pt += oofs;
    vert_it.get(i)->set_geom(pt);
  }
  std::vector<vtkPolyData*> poly_dats;
  //poly_dats.push_back(make_tessellation(body0));
  //poly_dats.push_back(make_tessellation(body1));
  auto booler = Boolean::ISolver::make();
  booler->init(body0, body1);
  auto inters = booler->compute(Boolean::Operation::DIFFERENCE);
  poly_dats.push_back(make_tessellation(inters));

  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/result.obj", inters);

  render_actors(poly_dats);
}

EXAMPLE(4)
{
  auto body0 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/cube.obj");
  auto body1 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/cube.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> vert_it(body1);
  const Geo::Vector3 oofs{ .5, .5, .5 };
  for (size_t i = 0; i < vert_it.size(); ++i)
  {
    Geo::Point pt;
    vert_it.get(i)->geom(pt);
    pt += oofs;
    vert_it.get(i)->set_geom(pt);
  }
  std::vector<vtkPolyData*> poly_dats;
#if 0
  poly_dats.push_back(make_tessellation(body0));
  poly_dats.push_back(make_tessellation(body1));
#else
  auto booler = Boolean::ISolver::make();
  booler->init(body0, body1);
  auto inters = booler->compute(Boolean::Operation::INTERSECTION);
  poly_dats.push_back(make_tessellation(inters));
#endif
  render_actors(poly_dats);
}

EXAMPLE(5)
{
  auto body0 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/cube_00.obj");
  auto body1 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/cube_00.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> vert_it(body1);
  const Geo::Vector3 oofs{ .5, .5, .5 };
  for (size_t i = 0; i < vert_it.size(); ++i)
  {
    Geo::Point pt;
    vert_it.get(i)->geom(pt);
    pt += oofs;
    vert_it.get(i)->set_geom(pt);
  }
  std::vector<vtkPolyData*> poly_dats;
#if 0
  poly_dats.push_back(make_tessellation(body0));
  poly_dats.push_back(make_tessellation(body1));
#else
  auto booler = Boolean::ISolver::make();
  booler->init(body0, body1);
  auto inters = booler->compute(Boolean::Operation::DIFFERENCE);
  poly_dats.push_back(make_tessellation(inters));
  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/cube_00_out.obj", inters);
#endif
  render_actors(poly_dats);
}

EXAMPLE(6)
{
  auto body0 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA.OBJ");
  auto body1 = IO::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA.OBJ");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> vert_it(body1);
  const Geo::Vector3 oofs{ 0.01, 0.01, 0.01 };
  for (size_t i = 0; i < vert_it.size(); ++i)
  {
    Geo::Point pt;
    vert_it.get(i)->geom(pt);
    pt += oofs;
    vert_it.get(i)->set_geom(pt);
  }
  std::vector<vtkPolyData*> poly_dats;
#if 0
  poly_dats.push_back(make_tessellation(body0));
  poly_dats.push_back(make_tessellation(body1));
#else
  auto booler = Boolean::ISolver::make();
  booler->init(body0, body1);
  auto result = booler->compute(Boolean::Operation::DIFFERENCE);
  poly_dats.push_back(make_tessellation(result));
  IO::save_obj("C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/result.obj", result);
#endif
  render_actors(poly_dats);
}

EXAMPLE(7)
{
  std::vector<double> knots = { 0, 0, 1./64, 1./32, 0.0625, 0.125, 0.25, 0.5, 1, 1 };
  struct Function0 : public Geo::IBsplineFitting<2>::IFunction
  {
    const double coe_ = M_PI * 3 / 2;
    virtual Geo::Vector<2> evaluate(const double _t) const
    {
      return Geo::Vector<2>{sin(_t * coe_), cos(_t * coe_)};
    }
    virtual Geo::Vector<2> closest_point(
      const Geo::Vector<2>& _pt, const double) const
    {
      return _pt / Geo::length(_pt);
    }
  };
  struct Function1 : public Geo::IBsplineFitting<2>::IFunction
  {
    const double one_third = 1. / 3, two_third = 2. / 3;
    virtual Geo::Vector<2> evaluate(const double _t) const
    {
      if (_t < one_third)
        return Geo::Vector<2>{ _t * 3, 1 };
      if (_t > two_third)
        return Geo::Vector<2>{ 1. - _t, -1. };
      return Geo::Vector<2>{ 1., 3. - _t * 6. };
    }
    virtual Geo::Vector<2> closest_point(
      const Geo::Vector<2>& _pt, const double _t) const
    {
      if (_t < one_third)
      {
        double x = std::max(std::min(_pt[0], 1.), 0.);
        return Geo::Vector<2>{ x, 1. };
      }
      if (_t > two_third)
      {
        double x = std::max(std::min(_pt[0], 1.), 0.);
        return Geo::Vector<2>{ x, -1. };
      }
      double y = std::max(std::min(_pt[1], 1.), -1.);
      return Geo::Vector<2>{ 1., y };
    }
  };
  const Function1 func;

  auto bsp_fit = Geo::IBsplineFitting<2>::make();
  bsp_fit->init(2, knots, func);
  //bsp_fit->set_parameter_correction_iterations(4);
  //bsp_fit->set_samples_per_interval(256);
  bsp_fit->compute();

  Geo::Nub<Geo::Vector<2>, double> ev_nub;
  std::vector<Geo::Vector<2>> opt_ctr_pts(bsp_fit->X());
  ev_nub.init(opt_ctr_pts, knots);
  std::vector<vtkPolyData*> poly_dats = { vtkPolyData::New(), vtkPolyData::New() };

  vtkPoints* newPoints[2];
  vtkCellArray* newPolys[2];

  for (auto i : { 0, 1 })
  {
    newPoints[i] = vtkPoints::New();
    newPoints[i]->Allocate(130);
    newPolys[i] = vtkCellArray::New();
    newPolys[i]->Allocate(64);
  }

  vtkIdType idxs[4] = { 0, 2, 3, 1 };
  std::ofstream plot("table.txt");
  for (double x = 0; x <= 1.; x += 1. / 256)
  {
    const double t = knots.back() * x + knots.front() * (1 - x);
    Geo::Vector<2> pt[2];
    pt[0] = func.evaluate(t);
    ev_nub.eval(t, &pt[1], &pt[1] + 1);
    auto dd = pt[1] - pt[0];
    double dist = Geo::length(pt[1]) - 1;
    if (pt[1][1] > 0 && pt[1][0] < 0 || x == 0 || x == 1)
      dist = Geo::length(pt[1] - pt[0]);
#define SEP << " " <<
    plot << /*t SEP Geo::length(dd) SEP dd[0] SEP dd[1] SEP */
      dist << std::endl;
    for (auto i : { 0, 1 })
    {
      for (auto z : { 0., 0.1 })
        newPoints[i]->InsertNextPoint(pt[i][0], pt[i][1], 0.1* i + z);
      if (x > 0)
        newPolys[i]->InsertNextCell(4, idxs);
    }
    if (x > 0)
      for (auto& idx : idxs) idx += 2;
  }
  for (auto i : { 0, 1 })
  {
    poly_dats[i]->SetPoints(newPoints[i]);
    newPoints[i]->Delete();
    poly_dats[i]->SetPolys(newPolys[i]);
    newPolys[i]->Delete();
  }
  Geo::Vector3 cols[2] = { { 1, 0, 0 },{ 0, 0, 1 } };
  render_actors(poly_dats, cols);
}