# 实现PCA分析和法向量计算，并加载数据集中的文件进行验证

import open3d as o3d 
import os
import numpy as np
from pyntcloud import PyntCloud

# 功能：计算PCA的函数
# 输入：
#     data：点云，NX3的矩阵
#     correlation：区分np的cov和corrcoef，不输入时默认为False
#     sort: 特征值排序，排序是为了其他功能方便使用，不输入时默认为True
# 输出：
#     eigenvalues：特征值
#     eigenvectors：特征向量
def PCA(data, correlation=False, sort=True):
    # 作业1
    # 屏蔽开始
    # 获取 data的shape大小
    data = data.T 
    # 获取数据的平均值
    # average = np.mean(data, axis=0)
    data = data - data.mean(axis=1, keepdims=True)
    # 数据中心化处理， 标准化处理
    data_T = data.T
    H = np.matmul(data, data_T)
    # svd
    eigenvalues, eigenvectors, _  =  np.linalg.svd(H, full_matrices=True)

    # 屏蔽结束

    if sort:
        sort = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[sort]
        eigenvectors = eigenvectors[:, sort]

    return eigenvalues, eigenvectors


def main():
    # 指定点云路径
    # cat_index = 1 # 物体编号，范围是0-39，即对应数据集中40个物体
    # root_dir = '/Users/linear/Documents/PyProject/Python_3D_Point/data_depth_annotated/' # 数据集路径
    # cat = os.listdir(root_dir)
    # filename = os.path.join(root_dir, cat[cat_index],'train', cat[cat_index]+'_0001.ply') # 默认使用第一个点云

    # 加载原始点云
    filename = "/Users/linear/Documents/GitFile/3D_Ponit_py/HomeworkI/data/person/person_0001.txt"
    point_cloud_pynt = PyntCloud.from_file(filename,  names=['x','y','z','nx','ny','nz'])
    point_cloud_o3d = point_cloud_pynt.to_instance("open3d", mesh=False)
    # o3d.visualization.draw_geometries([point_cloud_o3d]) # 显示原始点云

    # 从点云中获取点，只对点进行处理
    points = point_cloud_pynt.points
    print('total points number is:', points.shape[0])

    # 用PCA分析点云主方向
    w, v = PCA(points)
    point_cloud_vector = v[:, 2] #点云主方向对应的向量
    print('the main orientation of this pointcloud is: ', point_cloud_vector)
    # TODO: 此处只显示了点云，还没有显示PCA
    # 循环计算每个点的法向量
    projected_points = np.dot(point, v[:, :2])
    projected_points = np.hstack([projected_points,
                    np.zeros((projected_points.shape[0],1))])
    projected_point_cloud_o3d = o3d.geometry.PointCloud()
    projected_point_cloud_o3d.points = o3d.utility.Vector3dVector(projected_points)
    o3d.visualization.draw_geometries([point_cloud_o3d])
    normals = []
    # 作业2
    # 屏蔽开始
    # 由于最近邻搜索是第二章的内容，所以此处允许直接调用open3d中的函数
    cloud_range = points.max(axis=0) - points.min(axis=0)
    radius = cloud_range.max() * 0.05
    for point in point_cloud_o3d.points:
        cnt, idxs, dists = pcd_tree.search_radius_vector_3d(point, radius)
        w, v = PCA(points[idxs])
        normal = v[:, -1]
        normals.append(normal)

    # 屏蔽结束
    normals = np.array(normals, dtype=np.float64)
    # TODO: 此处把法向量存放在了normals中
    point_cloud_o3d.normals = o3d.utility.Vector3dVector(normals)
    o3d.visualization.draw_geometries([point_cloud_o3d])


if __name__ == '__main__':
    main()
