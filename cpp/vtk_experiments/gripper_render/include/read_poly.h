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

/**
 * @brief Load a mesh geometry into a PolyData object, supports different
 * mesh file formats.
 *
 * @param fileName input mesh filename
 * @return vtkSmartPointer<vtkPolyData>
 */
vtkSmartPointer<vtkPolyData> readPolyData(std::string const& fileName);