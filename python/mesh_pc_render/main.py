# ===================================
# Given a mesh and a camera position, render as a point cloud in different ways
# ===================================

import trimesh
import numpy as np
import os
import argparse

from io_utils import savePCNPtoOFF, savePCNPtoPCD, savePCNPtoXYZ
from renderers import renderMeshBackFaceCull, renderMeshRayCast, renderMeshAll, renderMeshEGL

# mesh_folder = "/home/fbottarel/workspace/docker-shared-workspace/shape-completion-ros/results/real_ycb_objects_results/meshes/holdout_models_holdout_views"
# output_folder = "/home/fbottarel/workspace/docker-shared-workspace/shape-completion-ros/results/real_ycb_objects_results/completed_clouds"
# mesh_filename = "/home/fbottarel/workspace/mesh_pc_render/textured_simple.obj"
# output_filename_RAW = "output.xyz"
# output_filename_PCD = "output.pcd"
# camera_viewpoint = np.array([0.6, 0.6, 0.6])
# NUMBER_SAMPLED_POINTS = 2000

def render_object(obj, viewpoints, args, mesh_filename_ending, mesh_folder, output_folder):

    print("Processing object " + obj)
    mesh_filename = os.path.join(mesh_folder, obj, mesh_filename_ending)

    # Load the mesh

    mesh = trimesh.load(mesh_filename)

    for idx_viewpoint in range(viewpoints.shape[0]):

        # Parse camera viewpoint

        camera_viewpoint = viewpoints[idx_viewpoint,:]
        z_towards_mesh = args.z_towards_mesh

        # Parse cardinality of the PC

        n_of_points = args.samples

        #points, points_camera_frame = renderMeshAll(mesh, camera_viewpoint, n_of_points, z_towards_mesh)
        #points, points_camera_frame = renderMeshBackFaceCull(mesh, camera_viewpoint, n_of_points, z_towards_mesh)
        #points, points_camera_frame = renderMeshRayCast(mesh, camera_viewpoint, n_of_points, z_towards_mesh)
        points, points_camera_frame = renderMeshEGL(mesh, camera_viewpoint, n_of_points, z_towards_mesh)

        # Add noise if needed

        if args.add_noise:

            noise_mean = 0
            noise_sigma_sq = 0.0000111
            number_of_samples = 5

            noise_mean_vector = np.ones((3)) * noise_mean
            noise_cov_matrix = np.eye(3) * noise_sigma_sq

            for idx_sample in range(number_of_samples):

                # Perturb point cloud with additive gaussian noise
                noise_samples = np.random.multivariate_normal(noise_mean_vector, noise_cov_matrix, points_camera_frame.shape[0])

                noisy_output_filename_raw = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + '_' + str(idx_sample).zfill(3) + "_pc.xyz"
                noisy_output_filename_pcd = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + '_' + str(idx_sample).zfill(3) + "_pc.pcd"
                noisy_output_filename_off = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + '_' + str(idx_sample).zfill(3) + "_pc.off"

                points_camera_frame_noisy = points_camera_frame + noise_samples

                # Save the noisy pcs
                savePCNPtoPCD(points_camera_frame_noisy, noisy_output_filename_pcd)
                savePCNPtoOFF(points_camera_frame_noisy, noisy_output_filename_off)

        else:

            # Save the non-noisy pcs

            output_filename_raw = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + "_pc.xyz"
            output_filename_pcd = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + "_pc.pcd"
            output_filename_off = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + "_pc.off"

            savePCNPtoPCD(points_camera_frame, output_filename_pcd)
            savePCNPtoOFF(points_camera_frame, output_filename_off)

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

    mesh_filename_ending = "textured_simple.obj"
    # mesh_filename_ending = "_mean_shape.ply"

    # If required, sample a bunch of viewpoints

    viewpoints = np.empty((0,3), float)

    if args.multiview:

        number_of_viewpoints = 50

        # Sample a bunch of points on a sphere

        phi = np.random.uniform(0, 2*np.pi, size=(number_of_viewpoints, 1))
        costheta = np.random.uniform(-1, 1, size=(number_of_viewpoints, 1))
        theta = np.arccos(costheta)
        r = np.linalg.norm(np.array(args.viewpoint))


        vp_x = r * np.sin(theta) * np.cos(phi)
        vp_y = r * np.sin(theta) * np.sin(phi)
        vp_z = r * np.cos(theta)

        viewpoints = np.append(viewpoints, np.hstack((vp_x, vp_y, vp_z)), axis=0)

    else:

        viewpoints = np.append(viewpoints, np.reshape(np.array(args.viewpoint), (1,3)), axis=0)

    # Let's try the parallel approach

    import multiprocessing
    from joblib import Parallel, delayed
    from tqdm import tqdm

    # num_cores = multiprocessing.cpu_count()
    num_cores = 1
    obj_dirlist = tqdm(object_dirlist)

    Parallel(n_jobs=num_cores)(delayed(render_object)(obj,
                                                    viewpoints,
                                                    args,
                                                    mesh_filename_ending,
                                                    mesh_folder,
                                                    output_folder)
                                                    for obj in obj_dirlist)

    # for obj in object_dirlist:

    #     print("Processing object " + obj)
    #     #mesh_filename = os.path.join(mesh_folder, obj, obj) + mesh_filename_ending
    #     mesh_filename = os.path.join(mesh_folder, obj, mesh_filename_ending)

    #     # Load the mesh

    #     mesh = trimesh.load(mesh_filename)

    #     for idx_viewpoint in range(viewpoints.shape[0]):

    #         # Parse camera viewpoint

    #         camera_viewpoint = viewpoints[idx_viewpoint,:]
    #         z_towards_mesh = args.z_towards_mesh

    #         # Parse cardinality of the PC

    #         n_of_points = args.samples

    #         #points, points_camera_frame = renderMeshAll(mesh, camera_viewpoint, n_of_points, z_towards_mesh)
    #         #points, points_camera_frame = renderMeshBackFaceCull(mesh, camera_viewpoint, n_of_points, z_towards_mesh)
    #         points, points_camera_frame = renderMeshRayCast(mesh, camera_viewpoint, n_of_points, z_towards_mesh)

    #         # Add noise if needed

    #         if args.add_noise:

    #             noise_mean = 0
    #             noise_sigma_sq = 0.0000111
    #             number_of_samples = 5

    #             noise_mean_vector = np.ones((3)) * noise_mean
    #             noise_cov_matrix = np.eye(3) * noise_sigma_sq

    #             for idx_sample in range(number_of_samples):

    #                 # Perturb point cloud with additive gaussian noise
    #                 noise_samples = np.random.multivariate_normal(noise_mean_vector, noise_cov_matrix, points_camera_frame.shape[0])

    #                 noisy_output_filename_raw = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + '_' + str(idx_sample).zfill(3) + "_pc.xyz"
    #                 noisy_output_filename_pcd = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + '_' + str(idx_sample).zfill(3) + "_pc.pcd"
    #                 noisy_output_filename_off = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + '_' + str(idx_sample).zfill(3) + "_pc.off"

    #                 points_camera_frame_noisy = points_camera_frame + noise_samples

    #                 # Save the noisy pcs
    #                 savePCNPtoPCD(points_camera_frame_noisy, noisy_output_filename_pcd)
    #                 savePCNPtoOFF(points_camera_frame_noisy, noisy_output_filename_off)

    #         else:

    #             # Save the non-noisy pcs

    #             output_filename_raw = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + "_pc.xyz"
    #             output_filename_pcd = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + "_pc.pcd"
    #             output_filename_off = os.path.join(output_folder, obj) + '_' + str(idx_viewpoint).zfill(4) + "_pc.off"

    #             savePCNPtoPCD(points_camera_frame, output_filename_pcd)
    #             savePCNPtoOFF(points_camera_frame, output_filename_off)

if __name__ == "__main__":

    argparser = argparse.ArgumentParser(description="Render meshes as point clouds.")

    argparser.add_argument('--mesh_folder', dest='mesh_folder', type=str, default='./models', help='Root of the directory tree where the meshes lie')
    argparser.add_argument('--z_towards_mesh', dest='z_towards_mesh', action='store_true', default=True, help='Whether the Z axis of the camera must be directed at the mesh')
    argparser.add_argument('--z_away_mesh', dest='z_towards_mesh', action='store_false', default=False, help='Whether the Z axis of the camera must face away from the mesh')
    argparser.add_argument('--output_folder', dest='output_folder', type=str, default='./partial_pc', help='Root of the directory tree where the rendered point clouds will be stored')
    argparser.add_argument('--viewpoint', dest='viewpoint', nargs=3, type=float, default=[0.4, 0.4, 0.4], help='Origin point of the camera in cartesian coordinates. The Z axis will pass through this point and the mesh CoM')
    argparser.add_argument('--multi_view', dest='multiview', action='store_true', default=False, help='Will sample point clouds from multiple points of view. Distance from the CoM is the same as viewpoint')
    argparser.add_argument('--samples', dest='samples', type=int, default=3000, help='Number of points to sample from the rendered mesh')
    argparser.add_argument('--noise', dest='add_noise', action='store_true', default=False, help='Add Gaussian noise to the point cloud')

    args = argparser.parse_args()

    main(args)
