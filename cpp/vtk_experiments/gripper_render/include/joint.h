#ifndef JOINT_H
#define JOINT_H

#include <fstream>

#include "utils.h"
#include "object_mesh.h"
// #include "link.h"

#include <Eigen/Core>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>

namespace mev
{
    class Link;

    enum JointType { revolute,
                    prismatic,
                    fixed};

    class Joint
    {
        std::string joint_name;

        std::shared_ptr<mev::Link> parent_link;
        std::shared_ptr<mev::Link> child_link;

        mev::JointType type;

        double joint_value;
        Eigen::Matrix4f joint_ref_frame;
        Eigen::Matrix3f joint_axis;

    };
}
#endif