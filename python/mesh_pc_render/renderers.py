import trimesh
import numpy as np
import pyrender

def renderMeshAll(mesh, cam_viewpoint, number_sampled_points, z_towards_mesh=True):
    """
    Renders a number of points on a mesh from a viewpoint, not considering any occlusion.

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

    points_camera_frame: (n, 3) float
        Points sampled on the visible portion of the mesh, in the camera reference frame

    """

    mesh_centroid = mesh.centroid

    # z axis is the ray from camera to the mesh center
    # This inversion is necessary because the camera reference system uses the cinematographic
    # convention
    # https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function

    if z_towards_mesh:
        camera_z_axis = (cam_viewpoint - mesh_centroid) / np.linalg.norm(mesh.centroid - cam_viewpoint)
        camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), camera_z_axis)
    else:
        camera_z_axis = (mesh_centroid - cam_viewpoint) / np.linalg.norm(mesh.centroid - cam_viewpoint)
        camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), -camera_z_axis)

    camera_transform_4x4[0:3,3] = cam_viewpoint

    # Sample points on the mesh surface (roughly equidistant)

    points, _ = trimesh.sample.sample_surface_even(mesh, number_sampled_points)
    points_homogeneous = np.hstack((points, np.ones((points.shape[0],1), dtype=np.float32)))

    # Transform the points as the camera was in the origin of the mesh

    points_camera_frame_homogeneous = np.transpose(np.matmul(np.linalg.inv(camera_transform_4x4), np.transpose(points_homogeneous)))
    points_camera_frame = points_camera_frame_homogeneous[:,0:3]

    return points, points_camera_frame

def renderMeshBackFaceCull(mesh, cam_viewpoint, number_sampled_points, z_towards_mesh=True):
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

    points_camera_frame: (n, 3) float
        Points sampled on the visible portion of the mesh, in the camera reference frame

    """

    mesh_centroid = mesh.centroid

    # z axis is the ray from camera to the mesh center
    # This inversion is necessary because the camera reference system uses the cinematographic
    # convention
    # https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function

    if z_towards_mesh:
        camera_z_axis = (cam_viewpoint - mesh_centroid) / np.linalg.norm(mesh.centroid - cam_viewpoint)
        camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), camera_z_axis)
    else:
        camera_z_axis = (mesh_centroid - cam_viewpoint) / np.linalg.norm(mesh.centroid - cam_viewpoint)
        camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), -camera_z_axis)

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
        #cull_condition = False

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

    points = mesh.sample(number_sampled_points, return_index=False)
    points_homogeneous = np.hstack((points, np.ones((points.shape[0],1), dtype=np.float32)))

    # Transform the points as the camera was in the origin of the mesh

    points_camera_frame_homogeneous = np.transpose(np.matmul(np.linalg.inv(camera_transform_4x4), np.transpose(points_homogeneous)))
    points_camera_frame = points_camera_frame_homogeneous[:,0:3]

    return points, points_camera_frame

def renderMeshEGL(mesh, cam_viewpoint, number_sampled_points, z_towards_mesh=True):
    """
    Renders a number of points on a mesh from a viewpoint, considering occlusion.
    Faces not in sight are detected by casting rays from a camera viewpoint.
    Uses EGL hardware acceleration.

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

    points_camera_frame: (n, 3) float
        Points sampled on the visible portion of the mesh, in the camera reference frame

    """

    import os
    os.environ["PYOPENGL_PLATFORM"] = "egl"
    mesh = pyrender.Mesh.from_trimesh(mesh)

    scene = pyrender.Scene()
    scene.add(mesh)

    # Set up the camera and depth of field
    fov = np.pi/3.0
    height = 480
    width = 640
    fx = fy = width / 2.0 / np.tan(fov / 2.0)
    cx = width / 2.0
    cy = height / 2.0
    camera = pyrender.IntrinsicsCamera(fx=fx, fy=fy, cx=cx, cy=cy)

    # z axis is the ray from the mesh center to the camera axis center
    # This inversion is necessary because the camera reference system uses the cinematographic
    # convention
    # https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function

    mesh_centroid = mesh.centroid
    if z_towards_mesh:
        camera_z_axis = (cam_viewpoint - mesh_centroid) / np.linalg.norm(mesh.centroid - cam_viewpoint)
    else:
        camera_z_axis = (mesh_centroid - cam_viewpoint) / np.linalg.norm(mesh.centroid - cam_viewpoint)

    # Obtain the camera transformation as a 4x4 and add it

    camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), camera_z_axis)
    camera_transform_4x4[0:3,3] = cam_viewpoint
    scene.add(camera, pose=camera_transform_4x4)

    # Set up the light -- a single spot light in the same spot as the camera
    light = pyrender.SpotLight(color=np.ones(3), intensity=3.0,
                                innerConeAngle=np.pi/16.0)
    scene.add(light, pose=camera_transform_4x4)

    # Render the scene
    r = pyrender.OffscreenRenderer(width, height)
    color, depth = r.render(scene)

    # Clip off depth values where there's nothing
    mask = np.where(depth > 0)
    x = mask[1]
    y = mask[0]

    # Compute coordinates given depth and image pixel coordinates
    points_x = (x.astype(np.float32) - cx) / fx * depth[y,x]
    points_y = (y.astype(np.float32) - cy) / fy * depth[y,x]
    points_z = depth[y,x]
    points_camera_frame = np.vstack((points_x, points_y, points_z)).T

    # Downsample if necessary
    if points_camera_frame.shape[0] > number_sampled_points:
        choice = np.random.choice(points_camera_frame.shape[0], number_sampled_points, replace=False)
        points_camera_frame = points_camera_frame[choice, :]

    # Obtain point coordinates in the object-centric ref system
    points_cam_homogeneous = np.hstack((points_camera_frame, np.ones((points_camera_frame.shape[0],1), dtype=np.float32)))
    points_world_frame_homogeneous = np.transpose(np.matmul(np.linalg.inv(camera_transform_4x4), np.transpose(points_cam_homogeneous)))
    points_world_frame = points_world_frame_homogeneous[:, 0:3]

    return points_world_frame, points_camera_frame

def renderMeshRayCast(mesh, cam_viewpoint, number_sampled_points, z_towards_mesh=True):
    """
    Renders a number of points on a mesh from a viewpoint, considering occlusion.
    Faces not in sight are detected by casting rays from a camera viewpoint.
    Use pyembree for speedup.

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

    points_camera_frame: (n, 3) float
        Points sampled on the visible portion of the mesh, in the camera reference frame

    """

    mesh_centroid = mesh.centroid

    # z axis is the ray from the mesh center to the camera axis center
    # This inversion is necessary because the camera reference system uses the cinematographic
    # convention
    # https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function

    camera_z_axis = (cam_viewpoint - mesh_centroid) / np.linalg.norm(mesh.centroid - cam_viewpoint)

    # Obtain the camera transformation as a 4x4

    camera_transform_4x4 = trimesh.geometry.align_vectors(np.array([0,0,1], dtype=np.float32), camera_z_axis)
    camera_transform_4x4[0:3,3] = cam_viewpoint

    # Create a scene with the mesh and a camera pointing to it

    scene = mesh.scene()
    scene.set_camera(resolution=[640,480],
                     fov=[60, 45],
                     angles=trimesh.transformations.euler_from_matrix(camera_transform_4x4, 'sxyz'),
                     center=mesh_centroid,
                     distance=np.linalg.norm(camera_transform_4x4[0:3, 3]))

    # convert the camera to rays with one ray per pixel
    origins, vectors, pixels = scene.camera_rays()

    # do the actual ray- mesh queries
    points, index_ray, index_tri = mesh.ray.intersects_location(origins, vectors, multiple_hits=False)

    # downsample if necessary
    # WARNING: THIS MESSES UP index_ray AND index_tri
    if points.shape[0] > number_sampled_points:
        choice = np.random.choice(points.shape[0], number_sampled_points, replace=False)
        points = points[choice, :]

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

    if z_towards_mesh:
        camera_transform_4x4 = camera_transform_4x4 @ np.diag([1, -1, -1, 1])

    # Obtain point coordinates in the camera ref system

    points_camera_frame_homogeneous = np.transpose(np.matmul(np.linalg.inv(camera_transform_4x4), np.transpose(points_homogeneous)))
    points_camera_frame = points_camera_frame_homogeneous[:, 0:3]

    return points, points_camera_frame