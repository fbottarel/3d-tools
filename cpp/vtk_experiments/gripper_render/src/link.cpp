#include "link.h"

namespace mev
{
    Link::Link(urdf::LinkConstSharedPtr urdf_link, std::shared_ptr<mev::Link> parent_link)
    : urdf_link {urdf_link},
      parent_link {parent_link}
    {
        link_name = urdf_link->name;

        if (urdf_link->visual)
        {
            link_visual_origin = getHomogeneousTransform(urdf_link->visual->origin);
            // Only mesh geometry is supported atm
            if (urdf_link->visual->geometry->type == urdf_link->visual->geometry->MESH)
            {
                link_visual_geometry = std::make_shared<mev::URDFVisualGeometry> (std::dynamic_pointer_cast<urdf::Mesh>(urdf_link->visual->geometry));
            }
        }
        else
        {
            link_visual_origin = Eigen::Matrix4f::Identity();
        }

        // init joint element, if parent exists
        if (parent_link)
        {
            parent_to_link_joint = std::make_shared<mev::Joint> (urdf_link->parent_joint);
        }
    }

    std::string Link::getLinkName()
    {
        return link_name;
    }

    bool Link::linkHasParent()
    {
        if (parent_link)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Link::linkHasChildren()
    {
        if (children_link.size() > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void Link::setParentLink(std::shared_ptr<mev::Link> parent_link)
    {
        this->parent_link = parent_link;
    }

    void Link::addChildLink(std::shared_ptr<mev::Link> child_link)
    {
        // add child to vector
        children_link.push_back(child_link);
        // add joint from this link to child
        // children_link.add_joint(asd);
    }

    Eigen::Matrix4f Link::getTransformationToParentRefFrame()
    {
        if (!linkHasParent())
        {
            return Eigen::Matrix4f::Identity();
        }
        else
        {
            return parent_to_link_joint->getJointTransform();
        }
    }

    Eigen::Matrix4f Link::getAbsoluteLinkTransform()
    {
        // explore the tree backwards towards the root until parent=null
        Eigen::Matrix4f root_transform = Eigen::Matrix4f::Identity();
        std::shared_ptr<mev::Link> current_link = shared_from_this();
        while (current_link->linkHasParent())
        {
            root_transform = root_transform * getTransformationToParentRefFrame();
            current_link = parent_link;
        }
        return root_transform;
    }

    urdf::LinkConstSharedPtr Link::getURDFLink()
    {
        return urdf_link;
    }

}