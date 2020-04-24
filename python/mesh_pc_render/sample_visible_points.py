# ===================================
# Given a mesh and a camera position, implement a simple back face cull
# ===================================

import trimesh
import numpy as np
import os
#import PIL

MESH_FOLDER = "/home/fbottarel/workspace/3d-tools/python/mesh_pc_render/models"
OUT_DIRECTORY = "/home/fbottarel/workspace/3d-tools/python/mesh_pc_render/partial_pc"
MESH_OBJECT_FILENAME = "textured_simple.obj"
# MESH_FILENAME = "/home/fbottarel/workspace/mesh_pc_render/textured_simple.obj"
# OUTPUT_FILENAME_RAW = "output.xyz"
# OUTPUT_FILENAME_PCD = "output.pcd"
CAMERA_VIEWPOINT = np.array([0.6, 0.6, 0.6])
NUMBER_SAMPLED_POINTS = 500


def renderMeshBackFaceCull(mesh, cam_viewpoint, number_sampled_points):
    """
    Renders a number of points on a mesh from a viewpoint, considering occlusion.
    Faces not in sight are detected using a back face cull method. 

    Parameters
    ----------
    mesh: trimesh.Mesh
        Mesh to sample
    
    cam_viewpoint: (3,) float
        Point from which to observe the mesh

    number_sampled_points: int
        Number of points to sample on the mesh

    Returns
    ----------
    points: (n, 3) float
        Points sampled on the visible portion of the mesh, in the original mesh reference frame 

    points_transformed: (n, 3) float
        Points sampled on the visible portion of the mesh, in the camera reference frame

    """

    mesh_centroid = mesh.centroid

    # z axis is the ray from camera to the mesh center

    camera_z_axis = (mesh.centroid - cam_viewpoint) / np.linalg.norm(mesh.centroid - cam_viewpoint)

    camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), camera_z_axis)

    camera_transform_4x4[0:3,3] = cam_viewpoint

    # In order to understand which faces to remove from the mesh, we build a mask array

    mesh_total_faces = mesh.faces.shape[0]
    mesh_faces_to_keep = np.ones((mesh_total_faces), dtype=np.bool)

    for idx in range(0, mesh_total_faces):
        
        # Compute the cull condition here
        # Compute the ray from camera to center of the mesh face

        camera_face_vector = mesh.triangles_center[idx] - cam_viewpoint

        # Cull condition is the dot product between the segment and outbound surface normal

        cull_condition = np.dot(camera_face_vector, mesh.face_normals[idx]) > 0

        if cull_condition:
            mesh_faces_to_keep[idx] = False

    # Remove faces that do not meet the cull condition 

    mesh.update_faces(mesh_faces_to_keep)
    mesh.remove_unreferenced_vertices()

    # View the mesh in an OpenGL window
    # Generate a camera axis marker with z axis pointing to the mesh

    # camera_marker = trimesh.creation.axis(axis_length=0.05,
    #                             axis_radius=0.001,
    #                             origin_size=0.001,
    #                             transform=camera_transform_4x4)

    # (camera_marker + mesh).show()

    # Sample points over the remaining mesh surface

    points = mesh.sample(NUMBER_SAMPLED_POINTS, return_index=False)
    points_homogeneous = np.hstack((points, np.ones((points.shape[0],1), dtype=np.float32)))

    # Transform the points as the camera was in the origin of the mesh

    points_transformed_homogeneous = np.transpose(np.matmul(np.linalg.inv(camera_transform_4x4), np.transpose(points_homogeneous)))
    points_transformed = points_transformed_homogeneous[:,0:3]

    return points, points_transformed


def renderMeshRayCast(mesh, cam_viewpoint, number_sampled_points):
    """
    Renders a number of points on a mesh from a viewpoint, considering occlusion.
    Faces not in sight are detected by casting rays from a camera viewpoint. 

    Parameters
    ----------
    mesh: trimesh.Mesh
        Mesh to sample
    
    cam_viewpoint: (3,) float
        Point from which to observe the mesh

    number_sampled_points: int
        Number of points to sample on the mesh

    Returns
    ----------
    points: (n, 3) float
        Points sampled on the visible portion of the mesh, in the original mesh reference frame 

    points_transformed: (n, 3) float
        Points sampled on the visible portion of the mesh, in the camera reference frame

    """

    # z axis is the ray from the mesh center to the camera axis center
    # This inversion is necessary because the camera reference system uses the cinematographic
    # convention
    # https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function

    mesh_centroid = mesh.centroid
    camera_z_axis = (cam_viewpoint - mesh_centroid) / np.linalg.norm(mesh.centroid - cam_viewpoint)

    # Obtain the camera transformation as a 4x4

    camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), camera_z_axis)
    camera_transform_4x4[0:3,3] = cam_viewpoint

    # Create a scene with the mesh and a camera pointing to it

    scene = mesh.scene()
    scene.set_camera(resolution=[320,240],
                     fov=[60, 45], 
                     angles=trimesh.transformations.euler_from_matrix(camera_transform_4x4, 'sxyz'), 
                     center=mesh_centroid, 
                     distance=np.linalg.norm(camera_transform_4x4[0:3, 3]))

    # convert the camera to rays with one ray per pixel
    origins, vectors, pixels = scene.camera_rays()

    # do the actual ray- mesh queries
    points, index_ray, index_tri = mesh.ray.intersects_location(origins, vectors, multiple_hits=False)

    # for each hit, find the distance along its vector
    # depth = trimesh.util.diagonal_dot(points - origins[0],
    #                                   vectors[index_ray])
    # # find pixel locations of actual hits
    # pixel_ray = pixels[index_ray]

    # # create a numpy array we can turn into an image
    # # doing it with uint8 creates an `L` mode greyscale image
    # a = np.zeros(scene.camera.resolution, dtype=np.uint8)

    # # scale depth against range (0.0 - 1.0)
    # depth_float = ((depth - depth.min()) / depth.ptp())

    # # convert depth into 0 - 255 uint8
    # depth_int = (depth_float * 255).round().astype(np.uint8)
    # # assign depth to correct pixel locations
    # a[pixel_ray[:, 0], pixel_ray[:, 1]] = depth_int
    # # create a PIL image from the depth queries
    # img = PIL.Image.fromarray(a)

    # # # show the resulting image
    # img.show()

    points_homogeneous = np.hstack((points, np.ones((points.shape[0],1), dtype=np.float32)))

    # Transform the "cinematographic" camera axis triad in one with the z axis facing towards mesh

    camera_transform_4x4 = camera_transform_4x4 @ np.diag([1, -1, -1, 1])    

    # Obtain point coordinates in the camera ref system

    points_transformed_homogeneous = np.transpose(np.matmul(np.linalg.inv(camera_transform_4x4), np.transpose(points_homogeneous)))
    points_transformed = points_transformed_homogeneous[:, 0:3]

    return points, points_transformed


def savePCNPtoPCD(points, filename):
    """
    Save a numpy point vector to PCD format. 

    Parameters
    ----------
    points: (n, 3) float
        Points to save
    
    filename: str
        Name of the output file (with or without extension) as full path

    Returns
    ----------
    success: bool
        True if the operation was successful

    """

    if os.path.splitext(filename)[1] != ".pcd":
        filename += ".pcd"

    success = False

    pcd_file_header = (
    "# .PCD v.7 - Point Cloud Data file format\n"
    "VERSION .7\n"
    "FIELDS x y z\n"
    "SIZE 4 4 4\n"
    "TYPE F F F\n"
    "COUNT 1 1 1\n"
    "WIDTH " + str(points.shape[0])+"\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS " + str(points.shape[0])+"\n"
    "DATA ascii")

    try: 
        with open(filename, 'w') as handle:
            np.savetxt(handle, points, header=pcd_file_header, comments='')
            success = True
    except:
        print("Could not open file " + filename)

    return success


def savePCNPtoXYZ(points, filename):
    """
    Save a numpy point vector to raw text XYZ format. 

    Parameters
    ----------
    points: (n, 3) float
        Points to save
    
    filename: str
        Name of the output file (with or without extension) as full path

    Returns
    ----------
    success: bool
        True if the operation was successful

    """

    if os.path.splitext(filename)[1] != ".xyz":
        filename += ".xyz"

    success = False

    try: 
        with open(filename, 'w') as handle:
            np.savetxt(handle, points)
            success = True
    except:
        print("Could not open file " + filename)

    return success

def savePCNPtoOFF(points, filename):
    """
    Save a numpy point vector to text OFF file. 

    Parameters
    ----------
    points: (n, 3) float
        Points to save
    
    filename: str
        Name of the output file (with or without extension) as full path

    Returns
    ----------
    success: bool
        True if the operation was successful

    """

    if os.path.splitext(filename)[1] != ".off":
        filename += ".off"

    success = False

    off_file_header = (
    "OFF\n"
    + str(points.shape[0]) + " 0 0")

    try: 
        with open(filename, 'w') as handle:
            np.savetxt(handle, points, header=off_file_header, comments='')
            success = True
    except:
        print("Could not open file " + filename)

    return success


def main():

    # Load a bunch of meshes and sample visible points.
    # Suppose a dir tree: 
    # 
    # MESH_FOLDER
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
    

    if os.path.isdir(MESH_FOLDER):
        object_dirlist = os.listdir(MESH_FOLDER)
    if not os.path.isdir(OUT_DIRECTORY):
        os.mkdir(OUT_DIRECTORY)

    for obj in object_dirlist:

        #if "mustard" not in obj: continue

        print("Processing object " + obj)
        MESH_FILENAME = os.path.join(MESH_FOLDER, obj, MESH_OBJECT_FILENAME)
        OUTPUT_FILENAME_RAW = os.path.join(OUT_DIRECTORY, obj) + ".xyz"
        OUTPUT_FILENAME_PCD = os.path.join(OUT_DIRECTORY, obj) + "_pc.pcd"
        OUTPUT_FILENAME_OFF = os.path.join(OUT_DIRECTORY, obj) + ".off"

        # Load the mesh

        mesh = trimesh.load(MESH_FILENAME)

        # points, points_transformed = renderMeshBackFaceCull(mesh, CAMERA_VIEWPOINT, NUMBER_SAMPLED_POINTS)
        points, points_transformed = renderMeshRayCast(mesh, CAMERA_VIEWPOINT, NUMBER_SAMPLED_POINTS)

        savePCNPtoPCD(points_transformed, OUTPUT_FILENAME_PCD)
        savePCNPtoOFF(points_transformed, OUTPUT_FILENAME_OFF)

if __name__ == "__main__":
    main()
