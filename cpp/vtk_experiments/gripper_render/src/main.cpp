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

#include <vtkPolyData.h>
#include <vtkSphereSource.h>


#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <stdio.h>

#include "object_mesh.h"
#include "utils.h"
#include "link.h"


/// Print joints and links related to a link (recursively)
void printChildrenJointsLinks(const std::string link_name, const urdf::ModelInterfaceSharedPtr model);

int main(int argc, char const *argv[])
{
    std::string urdf_filename = argv[1];

    if (!fileExists(urdf_filename))
    {
        std::cout << "There is no such URDF file" << std::endl;
        return -1;
    }

    urdf::ModelInterfaceSharedPtr gripper_urdf_model = urdf::parseURDFFile(urdf_filename);

    if (gripper_urdf_model)
        std::cout << "URDF file parsed successfully" << std::endl;
    else
    {
        std::cout << "URDF file failed to parse" << std::endl;
        return -1;
    }

    std::cout << "Robot name: \t\t" << gripper_urdf_model->getName() << std::endl;
    std::cout << "Root node name: \t" << gripper_urdf_model->getRoot()->name << std::endl;

    // Explore the tree as a sanity test
    std::cout << "Joints:" << std::endl;
    printChildrenJointsLinks(gripper_urdf_model->getRoot()->name, gripper_urdf_model);

    // Try to load the mesh
    vtkSmartPointer<vtkPolyData> hand_polydata = readPolyDataFromFile("meshes/visual/hand.stl");
    vtkSmartPointer<vtkPolyData> finger_polydata = readPolyDataFromFile("meshes/visual/finger.stl");

    // Display the mesh
    // TEMPORARY
    // TODO: do a gripper_render class with all that's needed
    // to load meshes and render them in the proper position
    std::string background_color = "MidnightBlue";
    vtkNew<vtkNamedColors> colors;

    vtkNew<vtkAxesActor> axes_actor;
    axes_actor->SetTotalLength(0.1,0.1,0.1);
    axes_actor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
    axes_actor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
    axes_actor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(5);
    axes_actor->SetOrigin(0,0,0);
    vtkNew<vtkRenderer> renderer;
    renderer->AddActor(axes_actor);

    mev::ObjectMesh hand(std::string("meshes/visual/hand.stl"));
    Eigen::Matrix4f tmp_mat;
    tmp_mat.setIdentity();
    hand.setObjectTransform(tmp_mat);
    renderer->AddActor(hand.getObjectActor());

    mev::ObjectMesh finger_1("meshes/visual/finger.stl");
    tmp_mat.setIdentity();
    tmp_mat(2,3) = 0.06;
    tmp_mat.block<2,2>(0,0) << -1, 0, 0, -1;
    finger_1.setObjectTransform(tmp_mat);
    renderer->AddActor(finger_1.getObjectActor());

    mev::ObjectMesh finger_2("meshes/visual/finger.stl");
    tmp_mat.setIdentity();
    tmp_mat(2,3) = 0.06;
    finger_2.setObjectTransform(tmp_mat);
    renderer->AddActor(finger_2.getObjectActor());

    renderer->SetBackground(colors->GetColor3d(background_color).GetData());
    vtkNew<vtkRenderWindow> render_window;
    render_window->SetSize(600, 600);
    render_window->AddRenderer(renderer);
    render_window->SetWindowName("Basic render");
    vtkNew<vtkRenderWindowInteractor> interactor;
    interactor->SetRenderWindow(render_window);
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor->SetInteractorStyle(style);
    render_window->Render();
    interactor->Initialize();
    interactor->Start();

    return EXIT_SUCCESS;

}

/// Print joints and links related to a link (recursively)
void printChildrenJointsLinks(const std::string link_name, const urdf::ModelInterfaceSharedPtr model)
{
    for (auto joint:model->getLink(link_name)->child_joints)
    {
        std::cout << "\tParent:\t" << link_name << std::endl;
        std::cout << "\tJoint:\t" << joint->name << std::endl;
        std::cout << "\tChild:\t"  << joint->child_link_name << std::endl;
        std::cout << "\tTransform:" << std::endl;

        urdf::Pose pose = joint->parent_to_joint_origin_transform;
        std::cout << getHomogeneousTransform(pose) << std::endl << std::endl;

        printChildrenJointsLinks(joint->child_link_name, model);
    }
}