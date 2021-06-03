#include "gripper.h"

namespace mev
{
    Gripper::Gripper()
    {
        gripper_name = "default_gripper";
        gripper_root_pose = Eigen::Matrix4f::Identity();
        gripper_root_link = nullptr;
        gripper_joints.clear();
    }

    Gripper::Gripper(const urdf::ModelInterfaceSharedPtr source_urdf) : Gripper()
    {
        initGripperFromURDF(source_urdf);
    }

    Gripper::Gripper(const std::string gripper_urdf_filename) : Gripper()
    {
        initGripperFromURDF(gripper_urdf_filename);
    }

    void Gripper::initGripperFromURDF(const urdf::ModelInterfaceSharedPtr source_urdf)
    {
        urdf_model = source_urdf;
        gripper_name = urdf_model->getName();
        gripper_root_link = std::make_shared<Link> (source_urdf->getRoot(), nullptr, urdf_path);
        // follow the tree recursively
        initLinkTree(gripper_root_link);
    }

    void Gripper::initGripperFromURDF(const std::string gripper_urdf_filename)
    {
        if (!fileExists(gripper_urdf_filename))
        {
            std::cout << "There is no such URDF file" << std::endl;
            return;
        }
        urdf_model = urdf::parseURDFFile(gripper_urdf_filename);
        if (urdf_model)
        {
            std::cout << "URDF file parsed successfully" << std::endl;
            urdf_path = gripper_urdf_filename;
            initGripperFromURDF(urdf_model);
        }
        else
        {
            std::cout << "URDF file failed to parse" << std::endl;
        }
    }

    void Gripper::initLinkTree(std::shared_ptr<Link> root_link)
    {
        // for every child of this link
        // call this function
        // joint is automagically called by the link constructor
        for (auto urdf_child_link : root_link->getURDFLink()->child_links)
        {
            std::shared_ptr<Link> child_link = std::make_shared<Link> (urdf_child_link, root_link, urdf_path);
            root_link->addChildLink(child_link);
            gripper_joints.push_back(child_link->getParentToLinkJoint());
            initLinkTree(child_link);
        }
        return;
    }
}