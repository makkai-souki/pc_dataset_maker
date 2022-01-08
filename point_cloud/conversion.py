# import numpy as np
# import pandas as pd
import open3d as o3d


def from_numpy_to_pcd(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def from_pcd_to_df(pcd):
    points = pcd.points
