#ifndef GRIPPER_H
#define GRIPPER_H

#include "utils.h"
#include "joint.h"
#include "link.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <urdf_model/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/link.h>

#include <fstream>

namespace mev
{
    class Gripper
    {
        std::string gripper_name;

        Eigen::Matrix4f gripper_root_pose;
        std::shared_ptr<Link> gripper_root_link;

        std::vector<std::shared_ptr<Joint>> gripper_joints;

        urdf::ModelInterfaceSharedPtr urdf_model;

        public:

        Gripper();
        Gripper(const urdf::ModelInterfaceSharedPtr source_urdf);
        Gripper(const std::string gripper_urdf_filename);
        void initGripperFromURDF(const urdf::ModelInterfaceSharedPtr source_urdf);
        void initGripperFromURDF(const std::string gripper_urdf_filename);
        void initLinkTree(const std::shared_ptr<Link> root_link);
        void setGripperRootPose(const Eigen::Matrix4f& root_pose);
        Eigen::Matrix4f getGripperRootPose();
        void addGripperGeometriesToRenderer(const vtkSmartPointer<vtkRenderer> renderer);
        void refreshGripperGeometries();
    };
}




#endif