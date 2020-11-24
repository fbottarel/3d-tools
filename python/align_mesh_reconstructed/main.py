import open3d as o3d
import trimesh as trimesh
import numpy as np
import os
import glob
import time
import argparse

point_cloud_directory = "./pcs"
gt_mesh_directory = "./gt_meshes"
mesh_directory = "./meshes"

point_cloud_format = '.pcd'
mesh_format = '.obj'
gt_mesh_format = '.obj'

spin_speed_factor = 0.0

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
    scene.graph.update(scene.graph.nodes_geometry[2], matrix=matrix)

if __name__ == "__main__":
    

    if not os.path.isdir(os.path.abspath(point_cloud_directory)):
        print('Error: point cloud directory not found')

    if not os.path.isdir(os.path.abspath(gt_mesh_directory)):
        print('Error: ground truth mesh directory not found')

    if not os.path.isdir(os.path.abspath(mesh_directory)):
        print('Error: completed mesh directory not found')

    point_cloud_file_list = glob.glob(os.path.abspath(point_cloud_directory) + '/*' + point_cloud_format)
    mesh_file_list = glob.glob(os.path.abspath(mesh_directory) + '/*' + mesh_format)
    gt_mesh_file_list = glob.glob(os.path.abspath(mesh_directory) + '/*' + gt_mesh_format)

    point_cloud_name_list = [os.path.splitext(os.path.basename(path))[0] for path in point_cloud_file_list]
    mesh_name_list = [os.path.splitext(os.path.basename(path))[0] for path in mesh_file_list]
    gt_mesh_name_list = [os.path.splitext(os.path.basename(path))[0] for path in gt_mesh_file_list]

    for pc_name in point_cloud_name_list:

        if pc_name not in mesh_name_list or pc_name not in gt_mesh_name_list:
            continue

        mesh_name = pc_name
        gt_mesh_name = pc_name

        print('Loading ' + pc_name)

        # Read point cloud

        v = o3d.io.read_point_cloud(os.path.join(os.path.abspath(point_cloud_directory), pc_name) + point_cloud_format)
        pc = trimesh.PointCloud(np.asarray(v.points))

        # Load mesh

        mesh = trimesh.load_mesh(os.path.join(os.path.abspath(mesh_directory), mesh_name) + mesh_format)

        # Load ground truth mesh 

        gt_mesh = trimesh.load_mesh(os.path.join(os.path.abspath(gt_mesh_directory), gt_mesh_name) + mesh_format)

        # Change mesh vertices and faces color and set transparency

        mesh.visual.face_colors[:] = np.array([255, 50, 50, 80])
        mesh.visual.vertex_colors[:] = np.array([255, 50, 50, 80])

        # gt_mesh.visual.face_colors[:] = np.array([50, 50, 255, 80])
        # gt_mesh.visual.vertex_colors[:] = np.array([50, 50, 255, 80])

        # Rotate mesh and pc of a fixed amount

        transform = trimesh.transformations.rotation_matrix(-110*(np.pi/180), (1,0,0), (0,0,0))
        mesh.apply_transform(transform)
        pc.apply_transform(transform)

        # Align ground truth to completed mesh

        alignment_transform = trimesh.registration.mesh_other(mesh, pc.vertices, samples=500)[0]
        mesh.apply_transform(alignment_transform)

        # Show the pc and mesh

        scene = trimesh.Scene([pc, mesh, gt_mesh])
        scene.show(callback=rotation_callback)
