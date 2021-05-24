#include "object_mesh.h"

ObjectMesh::ObjectMesh(const std::string& path,
                       const std::string& color,
                       const Eigen::Vector3f& origin)
    : mesh_path{path},
      mesh_color{color}
{

    // Standard VTK procedure to produce a polydata actor

    object_polydata = readPolyDataFromFile(mesh_path);
    object_mapper->SetInputData(object_polydata);
    object_actor->SetMapper(object_mapper);
    object_actor->SetOrigin(origin[0], origin[1], origin[2]);     // This could be useful later
    vtkNew<vtkNamedColors> colors;
    object_actor->GetProperty()->SetColor(colors->GetColor3d(mesh_color).GetData());

}

vtkSmartPointer<vtkActor> ObjectMesh::getObjectActor()
{
    return object_actor;
}

bool ObjectMesh::setObjectTransform(const Eigen::Matrix4f& transform4x4)
{
    Eigen::Matrix3f rotation = transform4x4.block<3,3>(0,0);
    if (rotation.determinant() != 1.0f)
    {
        std::cout << "Rotation matrix has determinant != 1" << std::endl;
        return false;
    }

    if (transform4x4(3,3) != 1.0f)
    {
        std::cout << "4x4 matrix is not homogeneous" << std::endl;
        return false;
    }

    Eigen::Vector4f translation = transform4x4.col(3);
    object_actor->SetPosition(translation[0],
                              translation[1],
                              translation[2]);
    // Express rotation as zxy
    Eigen::Vector3f rotation_euler_deg = rotation.eulerAngles(2,0,1) * 180/M_PI;
    // VTK requires rotations input as x y and z, in degrees
    object_actor->SetOrientation(rotation_euler_deg[1],
                                 rotation_euler_deg[2],
                                 rotation_euler_deg[0]);

    return true;

}
