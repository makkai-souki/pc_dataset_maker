import open3d as o3d
import numpy as np


class PointSeparator():
    def __init__(self, num_points, mode):
        self.num_points = num_points
        self.mode = mode

    def get_near_points(self, points, x, y, z):
        if self.mode == 'ns':
            return self.sphere_pick(points, x, y, z, normalize=True)
        elif self.mode == 'nc':
            return self.circle_pick(points, x, y, z, normalize=True)
        elif self.mode == 's':
            return self.sphere_pick(points, x, y, z, normalize=False)
        elif self.mode == 'c':
            return self.circle_pick(points, x, y, z, normalize=False)

    def sphere_pick(self, points, x, y, z, normalize=False):
        points['norm'] = (points['x'] - x) ** 2 + \
        (points['y'] - y) ** 2 + (points['z'] - z) ** 2
        points = points.sort_values('norm')[0:self.num_points]
        points['rel_x'] = points['x'] - x
        points['rel_y'] = points['y'] - y
        points['rel_z'] = points['z'] - z
        if normalize:
            points = self.normalize_points(points)
        return points

    def circle_pick(self, points, x, y, z, normalize=False):
        points['norm'] = (points['x'] - x) ** 2 + \
        (points['y'] - y) ** 2
        points = points.sort_values('norm')[0:self.num_points]
        points['rel_x'] = points['x'] - x
        points['rel_y'] = points['y'] - y
        points['rel_z'] = points['z'] - z
        if normalize:
            points = self.normalize_points(points)
        return points

    def normalize_points(self, points):
        points['rel_x'] = (points['rel_x'] - points['rel_x'].mean()) / points['rel_x'].std()
        points['rel_y'] = (points['rel_y'] - points['rel_y'].mean()) / points['rel_y'].std()
        points['rel_z'] = (points['rel_z'] - points['rel_z'].mean()) / points['rel_z'].std()
        return points



def get_near_points(points, x, y, z, *, x_name='x',
                    y_name='y', z_name='z', n=64, relative=True):
    """ 座標(x,y,z)から距離の近い点n個を抽出する関数

    Args:
        points (Pandas DataFrame): 元の点群
        x (float): 基準座標x
        y (float):  基準座標y
        z (float):  基準座標z
        x_name (str, optional): 基準座標xのDataFrameにおけるカラム名. Defaults to 'x'.
        y_name (str, optional): 基準座標yのDataFrameにおけるカラム名. Defaults to 'y'.
        z_name (str, optional): 基準座標yのDataFrameにおけるカラム名. Defaults to 'z'.
        n (int, optional): 抽出する点群数. Defaults to 64.

    Returns:
        Pandas DataFrame: 点群データフレーム（距離付き）
    """
    points['norm'] = (points[x_name] - x) ** 2 + \
        (points[y_name] - y) ** 2 + (points[z_name] - z) ** 2
    if relative:
        points['rel_' + x_name] = points[x_name] - x
        points['rel_' + y_name] = points[y_name] - y
        points['rel_' + z_name] = points[z_name] - z
    return points.sort_values('norm')[0:n]


def get_near_points_circle(points, x, y, z, *, x_name='x',
                    y_name='y', z_name='z', n=64, relative=True):
    points['norm'] = (points[x_name] - x) ** 2 + \
        (points[y_name] - y) ** 2
    if relative:
        points['rel_' + x_name] = points[x_name] - x
        points['rel_' + y_name] = points[y_name] - y
        points['rel_' + z_name] = points[z_name] - z
    return points.sort_values('norm')[0:n]


def estimate_normals(pcd):
    """ 法線推定

    Args:
        pcd (pcd(open3d)): 3次元点群オブジェクト

    Returns:
        pcd(open3d): 法線付き3次元点群オブジェクト
    """
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                          max_nn=30)
    )
    return pcd


def pick_points(pcds):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    for pcd in pcds:
        vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def manual_registration(source, target):
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    assert(len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert(len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.registration.TransformationEstimationPointToPoint()
    source = p2p.compute_transformation(source, target,
                                        o3d.utility.Vector2iVector(corr))
    return source, o3d.utility.Vector2iVector(corr)


def import_geometry_file(filename):
    return o3d.io.read_point_cloud(filename)
