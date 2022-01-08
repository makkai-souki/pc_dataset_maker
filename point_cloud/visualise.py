import open3d as o3d
import copy
import configparser


config_ini = configparser.ConfigParser()
config_ini.read('config.ini', encoding='utf-8')


def draw_registration_result(source, target, transformation):
    """ 位置合わせを描画する関数

    Args:
        source (geometry): 変換するジオメトリ
        target (geometry): 位置合わせ目標となるジオメトリ
        transformation (ndarray): 変換行列
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color(config_ini['DEFAULT']['YELLOW'])
    target_temp.paint_uniform_color(config_ini['DEFAULT']['BLUE'])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])


def draw_geometries(geometries, window_name='open3d'):
    """ 複数のopen3dジオメトリを描画する関数

    Args:
        geometries (list(geometry)): 描画するジオメトリのリスト
    """
    o3d.visualization.draw_geometries(geometries, window_name=window_name)


def crop_geometries(geometries, window_name='open3d'):
    """ 複数のopen3dジオメトリを描画する関数

    Args:
        geometries (list(geometry)): 描画するジオメトリのリスト
    """
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    o3d.visualization.draw_geometries_with_editing(geometries)
