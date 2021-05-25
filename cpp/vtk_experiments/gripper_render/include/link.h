#ifndef LINK_H
#define LINK_H

#include <fstream>

#include "utils.h"
#include "object_mesh.h"
#include "joint.h"

#include <Eigen/Core>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>

namespace mev
{
    class Link
    {
        std::string link_name;

        std::shared_ptr<mev::Link> parent_link; // not sure if useful
        std::vector<std::shared_ptr<mev::Link>> children_link; // not sure if useful

        Eigen::Matrix4f link_visual_origin;

        ObjectMesh link_object_mesh;

        // std::shared_ptr<mev::Joint> parent_to_link_joint;

        public:

        void setLinkVisual(const ObjectMesh& link_mesh);
        bool linkHasChildren();
        bool linkHasParent();
        Eigen::Matrix4f getAbsoluteLinkTransform(); // this requires tree exploration until linkHasParent is false

    };
}
#endif