import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy

# bunny = o3d.data.BunnyMesh()
# mesh = o3d.io.read_triangle_mesh(bunny.path)
# mesh.compute_vertex_normals()

#pcd = mesh.sample_points_poisson_disk(750)
# filter the point clouds and remove outliers
#print("Load a ply point cloud, print it, and render it")
#sample_pcd_data = o3d.data.PCDPointCloud()
# pcd = o3d.io.read_point_cloud("final.ply")
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# print("Downsample the point cloud with a voxel of 0.02")
# voxel_down_pcd = pcd.voxel_down_sample(voxel_size=5.0)
# o3d.visualization.draw_geometries([voxel_down_pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# print("Every 5th points are selected")
# uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
# o3d.visualization.draw_geometries([uni_down_pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])


# # 3. Vertex normal estimation Note: You can press N to show the normals and -/+ to control the length of the normals
# uni_down_pcd.estimate_normals(
# search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# o3d.visualization.draw_geometries([uni_down_pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024],
#                                   point_show_normal=True)
# # estimate_normals computes the normal for every point. The function finds adjacent points and calculates the principal axis of the adjacent points using covariance analysis. The function takes an instance of KDTreeSearchParamHybrid class as an argument. 
# # The two key arguments radius = 0.1 and max_nn = 30 specifies search radius and maximum nearest neighbor. It has 10cm of search radius, and only considers up to 30 neighbors to save computation time.

# # 4. Access estimated vertex normal
# print("Print a normal vector of the 0th point")
# print(uni_down_pcd.normals[0])

# print("Print the normal vectors of the first 10 points")
# print(np.asarray(uni_down_pcd.normals)[:10, :])


# def display_inlier_outlier(cloud, ind):
#     inlier_cloud = cloud.select_by_index(ind)
#     outlier_cloud = cloud.select_by_index(ind, invert=True)

#     print("Showing outliers (red) and inliers (gray): ")
#     outlier_cloud.paint_uniform_color([1, 0, 0])
#     inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
#     o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
#                                       zoom=0.3412,
#                                       front=[0.4257, -0.2125, -0.8795],
#                                       lookat=[2.6172, 2.0475, 1.532],
#                                       up=[-0.0694, -0.9768, 0.2024])


# print("Statistical oulier removal")
# cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=50,
#                                                     std_ratio=2.0)
# display_inlier_outlier(voxel_down_pcd, ind)


#  Method 1: Alpha shape
# pcd = o3d.io.read_point_cloud("final.ply", format='auto', remove_nan_points=False, remove_infinite_points=False, print_progress=False)
# alpha = 0.02
# print(f"alpha={alpha:.3f}")
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
# mesh.compute_vertex_normals()
# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
# for alpha in np.logspace(np.log10(0.5), np.log10(0.01), num=4):
#     print(f"alpha={alpha:.3f}")
#     mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
#         pcd, alpha, tetra_mesh, pt_map)
#     mesh.compute_vertex_normals()
#     o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# # Method 2: Ball Pivoting
# pcd = o3d.io.read_point_cloud("final.ply", format='auto', remove_nan_points=False, remove_infinite_points=False, print_progress=False)
# radii = [0.005, 0.01, 0.02, 0.04]
# rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
# pcd, o3d.utility.DoubleVector(radii))
# o3d.visualization.draw_geometries([pcd, rec_mesh])



# Method 3: Poisson Reconstruction
pcd = o3d.io.read_point_cloud("final.ply", format='auto', remove_nan_points=False, remove_infinite_points=False, print_progress=False)

print("Paint chair")
pcd.paint_uniform_color([136/255.0, 140/255.0, 141/255.0])
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])

print(pcd)
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])

print('run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)
print(mesh)
o3d.visualization.draw_geometries([mesh],
                                  zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])

o3d.io.write_triangle_mesh("final_mesh.obj", mesh)