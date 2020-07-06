# ===================================
# Given a mesh and a camera position, render as a point cloud in different ways
# ===================================

import trimesh
import numpy as np
import os
import argparse

from io_utils import savePCNPtoOFF, savePCNPtoPCD, savePCNPtoXYZ
from renderers import renderMeshBackFaceCull, renderMeshRayCast

# mesh_folder = "/home/fbottarel/workspace/docker-shared-workspace/shape-completion-ros/results/real_ycb_objects_results/meshes/holdout_models_holdout_views"
# output_folder = "/home/fbottarel/workspace/docker-shared-workspace/shape-completion-ros/results/real_ycb_objects_results/completed_clouds"
# mesh_filename = "/home/fbottarel/workspace/mesh_pc_render/textured_simple.obj"
# output_filename_RAW = "output.xyz"
# output_filename_PCD = "output.pcd"
# camera_viewpoint = np.array([0.6, 0.6, 0.6])
# NUMBER_SAMPLED_POINTS = 2000


def main(args):

    # Load a bunch of meshes and sample visible points.
    # Suppose a dir tree with a directory for each object, containing a mesh file:
    #
    # mesh_folder
    # ├── 002_master_chef_can
    # │   ├── 002_master_chef_can.xml
    # │   ├── points.xyz
    # │   ├── textured.mtl
    # │   ├── textured.obj
    # │   ├── textured_simple.obj
    # │   ├── textured_simple.obj.mtl
    # │   └── texture_map.png
    # ├── 004_sugar_box
    # │   ├── 004_sugar_box.xml
    # │   ├── points.xyz
    # │   ├── textured.mtl
    # │   ├── textured.obj
    # │   ├── textured_simple.obj
    # │   ├── textured_simple.obj.mtl
    # │   └── texture_map.png


    mesh_folder = args.mesh_folder

    if os.path.isdir(mesh_folder):
        object_dirlist = os.listdir(mesh_folder)
    
    output_folder = args.output_folder

    if not os.path.isdir(output_folder):
        os.mkdir(output_folder)

    mesh_filename_ending = "_mean_shape.ply"

    for obj in object_dirlist:

        print("Processing object " + obj)
        mesh_filename = os.path.join(mesh_folder, obj, obj) + mesh_filename_ending
        output_filename_raw = os.path.join(output_folder, obj) + ".xyz"
        output_filename_pcd = os.path.join(output_folder, obj) + "_pc.pcd"
        output_filename_off = os.path.join(output_folder, obj) + ".off"

        # Load the mesh

        mesh = trimesh.load(mesh_filename)

        # Parse camera viewpoint

        camera_viewpoint = args.viewpoint
        z_towards_mesh = args.z_towards_mesh

        # Parse cardinality of the PC

        n_of_points = args.samples

        points, points_camera_frame = renderMeshBackFaceCull(mesh, camera_viewpoint, n_of_points, z_towards_mesh)
        #points, points_camera_frame = renderMeshRayCast(mesh, camera_viewpoint, n_of_points, z_towards_mesh)


        savePCNPtoPCD(points_camera_frame, output_filename_pcd)
        savePCNPtoOFF(points_camera_frame, output_filename_off)

if __name__ == "__main__":

    argparser = argparse.ArgumentParser(description="Render meshes as point clouds.")

    argparser.add_argument('--mesh_folder', dest='mesh_folder', type=str, default='./models', help='Root of the directory tree where the meshes lie')
    argparser.add_argument('--z_towards_mesh', dest='z_towards_mesh', action='store_true', default=True, help='Whether the Z axis of the camera must be directed at the mesh')
    argparser.add_argument('--z_away_mesh', dest='z_towards_mesh', action='store_false', default=False, help='Whether the Z axis of the camera must face away from the mesh')
    argparser.add_argument('--output_folder', dest='output_folder', type=str, default='./partial_pc', help='Root of the directory tree where the rendered point clouds will be stored')
    argparser.add_argument('--viewpoint', dest='viewpoint', nargs=3, type=float, default=[0.6, 0.6, 0.6], help='Origin point of the camera in cartesian coordinates. The Z axis will pass through this point and the mesh CoM')
    argparser.add_argument('--samples', dest='samples', type=int, default=2000, help='Number of points to sample from the rendered mesh')

    args = argparser.parse_args()

    main(args)
