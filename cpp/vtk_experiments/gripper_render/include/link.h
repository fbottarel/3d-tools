#ifndef LINK_H
#define LINK_H

#include <fstream>

#include "utils.h"
#include "visual_geometry.h"
#include "joint.h"

#include <Eigen/Core>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>

namespace mev
{
    class Link : public std::enable_shared_from_this<mev::Link>
    {
        std::string link_name;

        std::shared_ptr<mev::Link> parent_link;
        std::vector<std::shared_ptr<mev::Link>> children_link;

        Eigen::Matrix4f link_visual_origin;
        std::shared_ptr<mev::URDFVisualGeometry> link_visual_geometry;

        urdf::LinkConstSharedPtr urdf_link;

        // std::shared_ptr<mev::Joint> joint_to_parent_link;

        public:

        Link(urdf::LinkConstSharedPtr urdf_link, std::shared_ptr<mev::Link> parent_link = nullptr);
        std::string getLinkName();
        bool linkHasChildren();
        bool linkHasParent();
        void setParentLink(std::shared_ptr<mev::Link> parent_link);
        // void setParentJoint(std::shared_ptr<mev::Joint> joint_to_parent_link);
        void addChildLink(std::shared_ptr<mev::Link> child_link);
        Eigen::Matrix4f getAbsoluteLinkTransform(); // this requires tree exploration until linkHasParent is false
    };
}
#endif