import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy

# # 1. Step 1: Given initial roto-translation, align the point clouds
# def draw_registration_result(source, target, transformation):
#     source_temp = copy.deepcopy(source)
#     target_temp = copy.deepcopy(target)
#     source_temp.paint_uniform_color([1, 0, 0])
#     target_temp.paint_uniform_color([0, 1, 0])
#     source_temp.transform(transformation)
#     o3d.visualization.draw_geometries([source_temp, target_temp],
#                                       zoom=0.4459,
#                                       front=[0.9288, -0.2951, -0.2242],
#                                       lookat=[1.6784, 2.0612, 1.4451],
#                                       up=[-0.3402, -0.9189, -0.1996])
#     o3d.io.write_point_cloud("result.ply",source_temp + target_temp)

# demo_icp_pcds = o3d.data.DemoICPPointClouds()
# # source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
# # target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

# source = o3d.io.read_point_cloud("0.ply")
# target = o3d.io.read_point_cloud("1.ply")


# print("Downsample the point cloud with a voxel of 0.05")

# source = source.voxel_down_sample(voxel_size=0.05)
# target = target.voxel_down_sample(voxel_size=0.05)
                    
# o3d.visualization.draw_geometries([source],
#                                   zoom=0.4459,
#                                       front=[0.9288, -0.2951, -0.2242],
#                                       lookat=[1.6784, 2.0612, 1.4451],
#                                       up=[-0.3402, -0.9189, -0.1996])
# o3d.visualization.draw_geometries([target],
#                                   zoom=0.4459,
#                                   front=[0.9288, -0.2951, -0.2242],
#                                   lookat=[1.6784, 2.0612, 1.4451],
#                                   up=[-0.3402, -0.9189, -0.1996])


# trans_init = np.asarray([[0.77, 0, 0.64, 49.46],
#                          [0.01, 1.00, -0.01, -0.32],
#                          [-0.64, 0.01, 0.77, 3.07],
#                          [0.0, 0.0, 0.0, 1.0]])
# draw_registration_result(source, target, trans_init)

# Step 2: From the result, merge the third point cloud onto the result point cloud
# 1. Input Data
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([0, 0, 1.0])
    #target_temp.paint_uniform_color([0, 0, 1.0])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
    o3d.io.write_point_cloud("final.ply",source_temp + target_temp)

demo_icp_pcds = o3d.data.DemoICPPointClouds()
# source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
# target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

source = o3d.io.read_point_cloud("2.ply")
target = o3d.io.read_point_cloud("result.ply")


print("Downsample the point cloud with a voxel of 0.05")

source = source.voxel_down_sample(voxel_size=0.05)
target = target.voxel_down_sample(voxel_size=0.05)
                    
o3d.visualization.draw_geometries([source],
                                  zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
o3d.visualization.draw_geometries([target],
                                  zoom=0.4459,
                                  front=[0.9288, -0.2951, -0.2242],
                                  lookat=[1.6784, 2.0612, 1.4451],
                                  up=[-0.3402, -0.9189, -0.1996])


trans_init = np.asarray([[0.93, 0.05, -0.36, -23.89],
                         [-0.06, 1.00, -0.02, 3.17],
                         [0.36, 0.04, 0.93, -3.36],
                         [0.0, 0.0, 0.0, 1.0]])
draw_registration_result(source, target, trans_init)



# def apply_noise(pcd, mu, sigma):
#     noisy_pcd = copy.deepcopy(pcd)
#     points = np.asarray(noisy_pcd.points)
#     points += np.random.normal(mu, sigma, size=points.shape)
#     noisy_pcd.points = o3d.utility.Vector3dVector(points)
#     return noisy_pcd


# mu, sigma = 0, 0.1  # mean and standard deviation
# source_noisy = apply_noise(source, mu, sigma)

# print("Source PointCloud + noise:")
# o3d.visualization.draw_geometries([source_noisy],
#                                   zoom=0.4459,
#                                   front=[0.353, -0.469, -0.809],
#                                   lookat=[2.343, 2.217, 1.809],
#                                   up=[-0.097, -0.879, 0.467])


# threshold = 0.02
# print("Vanilla point-to-plane ICP, threshold={}:".format(threshold))
# p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane()
# reg_p2l = o3d.pipelines.registration.registration_icp(source_noisy, target,
#                                                       threshold, trans_init,
#                                                       p2l)

# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
# draw_registration_result(source, target, reg_p2l.transformation)

# threshold = 1.0
# print("Vanilla point-to-plane ICP, threshold={}:".format(threshold))
# p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane()
# reg_p2l = o3d.pipelines.registration.registration_icp(source_noisy, target,
#                                                       threshold, trans_init,
#                                                       p2l)

# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
# draw_registration_result(source, target, reg_p2l.transformation)

# print("Robust point-to-plane ICP, threshold={}:".format(threshold))
# loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
# print("Using robust loss:", loss)
# p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
# reg_p2l = o3d.pipelines.registration.registration_icp(source_noisy, target,
#                                                       threshold, trans_init,
#                                                       p2l)
# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
# draw_registration_result(source, target, reg_p2l.transformation)

