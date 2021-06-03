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
#include <vtkRenderer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf_model/link.h>
#include <utils.h>

#include <math.h>

namespace mev
{
    class VisualGeometry
    {
        public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // mandatory when using eigen with fixed size matrices

        std::string mesh_path;
        std::string mesh_color;

        Eigen::Matrix4f geometry_world_pose;

        vtkSmartPointer<vtkPolyData> geometry_polydata;
        vtkNew<vtkPolyDataMapper> geometry_mapper;
        vtkNew<vtkActor> geometry_actor;

        VisualGeometry(const std::string& path = "",
                       const std::string& color = "SlateGray");
        bool setGeometryWorldPose(const Eigen::Matrix4f& geometry_world_pose);
        vtkSmartPointer<vtkActor> getGeometryActor();
        void addGeometryToRenderer(vtkSmartPointer<vtkRenderer> renderer);
    };

    class URDFVisualGeometry : public VisualGeometry
    {
        public:

        urdf::MeshSharedPtr urdf_geometry;
        std::string urdf_path;

        URDFVisualGeometry(urdf::MeshSharedPtr urdf_geometry, const std::string& urdf_path = "");
    };

}

















#endif