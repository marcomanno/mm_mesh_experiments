//
// This simple example shows how to do basic rendering and pipeline
// creation using C++.
//
#include "vtkCylinderSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCamera.h"

#include "Topology/iterator.hh"
#include "Import/import.hh"

#include <map>
#include <iostream>

template<size_t idxT> void example_function();

typedef void(*ExamplePtr)();

static std::map<size_t, ExamplePtr>& example_table()
{
  static std::map<size_t, ExamplePtr> exmpls_;
  return exmpls_;
}
template <size_t nmbrT>
struct Examples
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
void render_actor(vtkPolyData *_ply_dat)
{
  // The mapper is responsible for pushing the geometry into the graphics
  // library. It may also do color mapping, if scalars or other attributes
  // are defined.
  //
  vtkPolyDataMapper *actor_map = vtkPolyDataMapper::New();
  actor_map->SetInputData(_ply_dat);

  // The actor is a grouping mechanism: besides the geometry (mapper), it
  // also has a property, transformation matrix, and/or texture map.
  // Here we set its color and rotate it -22.5 degrees.
  vtkActor *actor = vtkActor::New();
  actor->SetMapper(actor_map);
  actor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
  actor->RotateX(30.0);
  actor->RotateY(-45.0);

  // Create the graphics structure. The renderer renders into the
  // render window. The render window interactor captures mouse events
  // and will perform appropriate camera or actor manipulation
  // depending on the nature of the events.
  //
  vtkRenderer *ren1 = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer(ren1);
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  // Add the actors to the renderer, set the background and size
  //
  ren1->AddActor(actor);
  ren1->SetBackground(0.1, 0.2, 0.4);
  renWin->SetSize(200, 200);

  // We'll zoom in a little by accessing the camera and invoking a "Zoom"
  // method on it.
  ren1->ResetCamera();
  ren1->GetActiveCamera()->Zoom(1.5);
  renWin->Render();

  // This starts the event loop and as a side effect causes an initial render.
  iren->Start();

  // Exiting from here, we have to delete all the instances that
  // have been created.
  actor_map->Delete();
  actor->Delete();
  ren1->Delete();
  renWin->Delete();
  iren->Delete();
}

}

EXAMPLE(0)
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
  render_actor(poly_dat);
}

EXAMPLE(1)
{
  auto body = Import::load_obj(
    "C:/Users/marco/OneDrive/Documents/PROJECTS/polytriagnulation/mesh/TUNA.OBJ");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> vert_it(body);
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> all_verts;
  for (size_t i = 0; i < vert_it.size(); ++i)
    all_verts.push_back(vert_it.get(i));
  std::sort(all_verts.begin(), all_verts.end());
  all_verts.erase(std::unique(all_verts.begin(), all_verts.end()), all_verts.end());

  vtkPolyData *poly_dat = vtkPolyData::New();

  auto newPoints = vtkPoints::New();
  newPoints->Allocate(all_verts.size());
  for (auto v : all_verts)
  {
    Geo::Point pt;
    v->geom(pt);
    newPoints->InsertNextPoint(pt[0], pt[1], pt[2]);
  }
  poly_dat->SetPoints(newPoints);
  newPoints->Delete();

  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> face_it(body);
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
  render_actor(poly_dat);
}