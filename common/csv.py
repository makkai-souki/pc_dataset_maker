import pandas as pd
import numpy as np
import open3d as o3d
import os


def read_csv_to_pcd(file_name):
    xyz = np.loadtxt(file_name, delimiter=',')
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd


def add_lines(filename, data):
    start_line = 0
    if os.path.exists(filename):
        tmp = pd.read_csv(filename)
        start_line = len(tmp)
        data.to_csv(filename)
    else:
        data.to_csv(filename, mode='a', header=False)
    end_line = start_line + len(data)
    return start_line, end_line
