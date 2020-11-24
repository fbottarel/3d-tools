"""This script is supposed to show a point cloud and a mesh of the same object,
overlaying them with a transparency value.
The script will look for a mesh file and point cloud with the same name.
"""


import open3d as o3d
import trimesh as trimesh
import numpy as np
import os
import glob
import time
import argparse

# point_cloud_directory = '/home/fbottarel/workspace/PointSDF/pcs/real_ycb/processed_pcs'
# mesh_directory = '/home/fbottarel/workspace/PointSDF/meshes/ycb_real'
# point_cloud_directory = '/home/fbottarel/workspace/PointSDF/pcs/rendered_ycb/processed_pcs'
# mesh_directory = '/home/fbottarel/workspace/PointSDF/meshes/ycb_rendered'
# point_cloud_format = '.pcd'
# mesh_format = '.obj'
spin_speed_factor = 0

def rotation_callback(scene):
    """
    A callback passed to a scene viewer which will update
    transforms in the viewer periodically.
    Parameters
    -------------
    scene : trimesh.Scene
      Scene containing geometry
    """

    angle = time.time() * spin_speed_factor
    matrix = trimesh.transformations.rotation_matrix(angle, (0,1,0), (0,0,0))

    # Apply the transform to the nodes
    scene.graph.update(scene.graph.nodes_geometry[0], matrix=matrix)
    scene.graph.update(scene.graph.nodes_geometry[1], matrix=matrix)

def argument_parse():
    """
    Parsing arguments from command line.

    Returns
    -------
    args : argparser.Namespace
        Populated namespace with command-line parameters
    """

    argparser = argparse.ArgumentParser(description="Show meshes and corresponding point clouds in a 3D viewer.")

    argparser.add_argument('--mesh_dir', dest='mesh_directory', type=str, default='./meshes', help='Directory where meshes can be found')
    argparser.add_argument('--pc_dir', dest='pc_directory', type=str, default='./pcs', help='Directory where point clouds can be found')
    argparser.add_argument('--mesh_format', dest='mesh_format', type=str, default='obj', help='Mesh format. Example: --mesh_format obj')
    argparser.add_argument('--pc_format', dest='point_cloud_format', type=str, default='pcd', help='Point cloud format. Example: --pc_format pcd')
    argparser.add_argument('--spin_speed', dest='spin_speed_factor', type=float, default=0.8, help='Spinning velocity factor [0, infinite]')

    args = argparser.parse_args()

    return args

if __name__ == "__main__":

    # Parser arguments

    args = argument_parse()

    point_cloud_directory = args.pc_directory
    mesh_directory = args.mesh_directory
    point_cloud_format = '.' + args.point_cloud_format
    mesh_format = '.' + args.mesh_format
    spin_speed_factor = args.spin_speed_factor

    # Check for dir existence

    if not os.path.isdir(os.path.abspath(point_cloud_directory)):
        print('Error: point cloud directory not found')

    if not os.path.isdir(os.path.abspath(mesh_directory)):
        print('Error: mesh directory not found')

    point_cloud_file_list = glob.glob(os.path.abspath(point_cloud_directory) + '/*' + point_cloud_format)
    mesh_file_list = glob.glob(os.path.abspath(mesh_directory) + '/*' + mesh_format)

    point_cloud_name_list = [os.path.splitext(os.path.basename(path))[0] for path in point_cloud_file_list]
    mesh_name_list = [os.path.splitext(os.path.basename(path))[0] for path in mesh_file_list]

    # For every cloud, load a mesh only if a file with the same name exists

    for pc_name in point_cloud_name_list:

        if pc_name not in mesh_name_list:
            continue

        mesh_name = pc_name

        print('Loading ' + pc_name)

        # Read point cloud

        v = o3d.io.read_point_cloud(os.path.join(os.path.abspath(point_cloud_directory), pc_name) + point_cloud_format)
        pc = trimesh.PointCloud(np.asarray(v.points))

        # Load mesh

        mesh = trimesh.load_mesh(os.path.join(os.path.abspath(mesh_directory), mesh_name) + mesh_format)

        # Change mesh vertices and faces color and set transparency

        mesh.visual.face_colors[:] = np.array([255, 50, 50, 80])
        mesh.visual.vertex_colors[:] = np.array([255, 50, 50, 80])

        # Rotate mesh and pc of a fixed amount

        transform = trimesh.transformations.rotation_matrix(-110*(np.pi/180), (1,0,0), (0,0,0))
        mesh.apply_transform(transform)
        pc.apply_transform(transform)

        # Show the pc and mesh

        scene = trimesh.Scene([pc, mesh])
        scene.show(callback=rotation_callback)
