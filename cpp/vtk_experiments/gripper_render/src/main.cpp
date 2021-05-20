#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <stdio.h>

// Including cpp files is discouraged...
#include "read_poly.cpp"

/// Check for file existence
bool fileExists(const std::string filename);

/// Pose to Eigen matrix
Eigen::Matrix4f getHomogeneousTransform(const urdf::Pose &pose);

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

    // Explore the tree
    std::cout << "Joints:" << std::endl;
    printChildrenJointsLinks(gripper_urdf_model->getRoot()->name, gripper_urdf_model);

    return 0;
}

/// Check for file existence
bool fileExists(const std::string filename)
{
    std::ifstream filestream(filename);
    return filestream.good();
}

/// Pose to Eigen matrix
Eigen::Matrix4f getHomogeneousTransform(const urdf::Pose &pose)
{
    Eigen::Quaternion<float> rotation_quat(pose.rotation.w,
                                           pose.rotation.x,
                                           pose.rotation.y,
                                           pose.rotation.z);

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Matrix4f transformation;
    transformation.setIdentity();
    transformation.block<3,3>(0,0) = rotation;
    transformation.block<3,1>(0,3) = translation;

    return transformation;
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