#ifndef VISUAL_GEOMETRY_H
#define VISUAL_GEOMETRY_H

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

#include <urdf_model/link.h>
#include <utils.h>

#include <math.h>

namespace mev
{
    class VisualGeometry
    {
        public:

        std::string mesh_path;
        std::string mesh_color;

        Eigen::Matrix4f geometry_world_pose;

        vtkSmartPointer<vtkPolyData> geometry_polydata;
        vtkNew<vtkPolyDataMapper> geometry_mapper; // shared pointer and then initialize in constructor
        vtkNew<vtkActor> geometry_actor;

        VisualGeometry(const std::string& path = "",
                       const std::string& color = "SlateGray");
        bool setGeometryWorldPose(const Eigen::Matrix4f& geometry_world_pose);
        vtkSmartPointer<vtkActor> getGeometryActor();
    };

    class URDFVisualGeometry : public VisualGeometry
    {
        public:

        urdf::MeshSharedPtr urdf_geometry;

        URDFVisualGeometry(urdf::MeshSharedPtr urdf_geometry);
    };

}

















#endif