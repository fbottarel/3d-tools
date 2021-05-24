#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>

#include <Eigen/Core>

#include <utils.h>

#include <math.h>

class ObjectMesh
{
    std::string mesh_path;
    std::string mesh_color;
    Eigen::Matrix4f mesh_origin;
    Eigen::Matrix4f object_transform;
    vtkSmartPointer<vtkPolyData> object_polydata;
    vtkNew<vtkPolyDataMapper> object_mapper;
    vtkNew<vtkActor> object_actor;

    public:

    ObjectMesh(const std::string& path,
               const std::string& color = "SlateGray",
               const Eigen::Vector3f& origin = {0,0,0});

    vtkSmartPointer<vtkActor> getObjectActor();
    bool setObjectTransform(const Eigen::Matrix4f& transform4x4);
};