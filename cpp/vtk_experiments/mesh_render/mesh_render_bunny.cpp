#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

// Readers
#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>

#include <vtkPolyData.h>
#include <vtkSphereSource.h>

#include <algorithm> // For transform()
#include <cctype>    // For to_lower
#include <string>    // For find_last_of()

#include <array>

vtkSmartPointer<vtkPolyData> ReadPolyData(std::string const& fileName);

int main(int argc, char* argv[])
{

  // Set up necessary strings

  std::string mesh_filename = argv[1];
  std::string mesh_color = "SlateGray";
  std::string background_color = "MidnightBlue";

  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " filename" << std::endl;
    return EXIT_FAILURE;
  }

  //
  // We now parse a mesh from file and convert it into a PolyData source. The
  // instance of vtkPolyData is a source for the visualization pipeline, that
  // other filters can process.
  //

  vtkSmartPointer<vtkPolyData> mesh_polydata;
  mesh_polydata = ReadPolyData(mesh_filename);

  //
  // In this program, there is no further processing/filtering of the polydata.
  // The source feeds into a mapper, i.e. a vtkPolyDataMapper object in order to
  // map the polygonal data into graphics primitives. We connect the output of
  // the source to the input of the mapper.
  //

  vtkNew<vtkPolyDataMapper> mesh_mapper;
  mesh_mapper->SetInputData(mesh_polydata);
  // mesh_mapper->SetInputConnection(mesh_polydata->GetOutputPort());
  // This doesn't work for a PolyData source, it works for other sources.

  //
  // Create an actor to represent the mesh. The actor orchestrates rendering
  // of the mapper's graphics primitives. An actor also refers to properties
  // via a vtkProperty instance, and includes an internal transformation
  // matrix. We set this actor's mapper to be the PolyDataMapper we just created
  // above.
  //

  vtkNew<vtkNamedColors> colors;
  vtkNew<vtkActor> mesh_actor;
  mesh_actor->SetMapper(mesh_mapper);
  mesh_actor->SetOrigin(0,0,0);
  mesh_actor->GetProperty()->SetColor(colors->GetColor3d(mesh_color).GetData());
  // mesh_actor->GetProperty()->SetSpecular(0.6);
  // mesh_actor->GetProperty()->SetSpecularPower(30);

  //
  // Add axes in the center of the reference system.
  //

  vtkNew<vtkAxesActor> axes_actor;
  axes_actor->SetTotalLength(0.1,0.1,0.1);
  axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
  axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
  axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
  axes_actor->SetOrigin(0,0,0);

  //
  // Create the renderer and assign actors to it. A renderer is like a viewport,
  // it is part of a window and it is responsible for drawing the actors
  // assigned to it. Other information such as camera and background can be used
  // here.
  //

  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(axes_actor);
  renderer->AddActor(mesh_actor);
  renderer->SetBackground(colors->GetColor3d(background_color).GetData());

  //
  // Create the rendering window that will show up on the screen, set its size
  // and assign a renderer to it.
  //

  vtkNew<vtkRenderWindow> render_window;
  render_window->SetSize(600, 600);
  render_window->AddRenderer(renderer);
  render_window->SetWindowName("Basic render");

  //
  // The vtkRenderWindowInteractor class watches for events (e.g., keypress,
  // mouse) in the vtkRenderWindow. These events are translated into
  // event invocations that VTK understands (see VTK/Common/vtkCommand.h
  // for all events that VTK processes). Then observers of these VTK
  // events can process them as appropriate.
  //

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(render_window);

  //
  // By default the vtkRenderWindowInteractor instantiates an instance
  // of vtkInteractorStyle. vtkInteractorStyle translates a set of events
  // it observes into operations on the camera, actors, and/or properties
  // in the vtkRenderWindow associated with the vtkRenderWinodwInteractor.
  // Here we specify a particular interactor style.
  //

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  interactor->SetInteractorStyle(style);

  //
  // We render the actor and leave the loop running.
  //

  render_window->Render();
  interactor->Initialize();
  interactor->Start();

  return EXIT_SUCCESS;

}

vtkSmartPointer<vtkPolyData> ReadPolyData(std::string const& fileName)
{
  //
  // Load a mesh geometry into a PolyData object. This adapts to a bunch of
  // different mesh file formats.
  //

  vtkSmartPointer<vtkPolyData> polyData;
  std::string extension = "";
  if (fileName.find_last_of(".") != std::string::npos)
  {
    extension = fileName.substr(fileName.find_last_of("."));
  }
  // Make the extension lowercase
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  if (extension == ".ply")
  {
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".vtp")
  {
    vtkNew<vtkXMLPolyDataReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".obj")
  {
    vtkNew<vtkOBJReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".stl")
  {
    vtkNew<vtkSTLReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".vtk")
  {
    vtkNew<vtkPolyDataReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".g")
  {
    vtkNew<vtkBYUReader> reader;
    reader->SetGeometryFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else
  {
    // Return a polydata sphere if the extension is unknown.
    vtkNew<vtkSphereSource> source;
    source->SetThetaResolution(20);
    source->SetPhiResolution(11);
    source->Update();
    polyData = source->GetOutput();
  }
  return polyData;
}