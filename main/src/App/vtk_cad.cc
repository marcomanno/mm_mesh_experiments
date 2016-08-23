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

int main()
{
  // This creates a polygonal cylinder model with eight circumferential facets.
  //
  vtkCylinderSource *cylinder = vtkCylinderSource::New();
  cylinder->SetResolution(8);

  vtkPolyData *output = vtkPolyData::New();
  auto newPoints = vtkPoints::New();
  newPoints->Allocate(8);
  for (int idx = 0; idx < 8; ++idx)
  {
    double vv[2] = { -1, 1 };
	newPoints->InsertNextPoint(
      vv[(idx & 4) > 0], vv[(idx & 2) > 0], vv[(idx & 1) > 0]);
  }	
  output->SetPoints(newPoints);
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

  output->SetPolys(newPolys);
  newPolys->Delete();


  // The mapper is responsible for pushing the geometry into the graphics
  // library. It may also do color mapping, if scalars or other attributes
  // are defined.
  //
  vtkPolyDataMapper *cylinderMapper = vtkPolyDataMapper::New();
  cylinderMapper->SetInputData(output);
  //cylinderMapper->SetInputConnection(cylinder->GetOutputPort());

  // The actor is a grouping mechanism: besides the geometry (mapper), it
  // also has a property, transformation matrix, and/or texture map.
  // Here we set its color and rotate it -22.5 degrees.
  vtkActor *cylinderActor = vtkActor::New();
  cylinderActor->SetMapper(cylinderMapper);
  cylinderActor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
  cylinderActor->RotateX(30.0);
  cylinderActor->RotateY(-45.0);

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
  ren1->AddActor(cylinderActor);
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
  cylinder->Delete();
  cylinderMapper->Delete();
  cylinderActor->Delete();
  ren1->Delete();
  renWin->Delete();
  iren->Delete();

  return 0;
}
