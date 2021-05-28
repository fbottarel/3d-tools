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

#include "utils.h"
#include "joint.h"
#include "link.h"
#include "visual_geometry.h"


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

    urdf::LinkConstSharedPtr hand_link = gripper_urdf_model->getLink("panda_hand");
    urdf::LinkConstSharedPtr finger_left_link = gripper_urdf_model->getLink("panda_leftfinger");
    urdf::LinkConstSharedPtr finger_right_link = gripper_urdf_model->getLink("panda_rightfinger");

    Eigen::Matrix4f hand_tf(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f finger_right_tf(hand_tf);
    finger_right_tf(2,3) = 0.06;
    Eigen::Matrix4f finger_left_tf(finger_right_tf);
    finger_left_tf.block<2,2>(0,0) << -1, 0, 0, -1;

    // Dynamic casting is necessary since the urdf parsed model points to a parent class of urdf::Mesh
    mev::URDFVisualGeometry hand_geom(std::dynamic_pointer_cast<urdf::Mesh>(hand_link->visual->geometry));
    mev::URDFVisualGeometry finger_left_geom(std::dynamic_pointer_cast<urdf::Mesh>(finger_left_link->visual->geometry));
    mev::URDFVisualGeometry finger_right_geom(std::dynamic_pointer_cast<urdf::Mesh>(finger_right_link->visual->geometry));

    hand_geom.setGeometryWorldPose(hand_tf);
    finger_left_geom.setGeometryWorldPose(finger_left_tf);
    finger_right_geom.setGeometryWorldPose(finger_right_tf);

    renderer->AddActor(hand_geom.getGeometryActor());
    renderer->AddActor(finger_right_geom.getGeometryActor());
    renderer->AddActor(finger_left_geom.getGeometryActor());

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

    std::shared_ptr<mev::Link> root_link = std::make_shared<mev::Link> (hand_link);
    for (auto current_link : hand_link->child_links)
    {
        // create the link
        std::cout << "Adding child link " << current_link->name << std::endl;
        std::shared_ptr<mev::Link> child = std::make_shared<mev::Link> (current_link, root_link);
        root_link->addChildLink(child);
    }

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