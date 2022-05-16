from point_cloud.visualise import crop_geometries, draw_geometries
from point_cloud.conversion import from_numpy_to_pcd
from point_cloud.operations import PointSeparator, manual_registration, pick_points
from common.csv import read_csv_to_pcd
import configparser
import json
import os
import glob
import open3d as o3d
import numpy as np
import pandas as pd


class Run(object):
    def __init__(self):
        self.main_app = MainApp()

    def test(self):
        print("test")

    def annotation(self, before_name, after_name):
        self.main_app.set_variation_names(before_name, after_name)
        self.main_app.import_raw_pointclouds()
        self.main_app.draw_variation()
        self.main_app.crop_loop()

    def registration(self, before_name, after_name):
        self.main_app.set_variation_names(before_name, after_name)
        self.main_app.import_raw_pointclouds()
        self.main_app.registrate_variation()

    def split(self, collapse_name, reference_mode='r', num_points=64, ref_path=None, 
              edge_dir=None, freq=64, relative=True, separate_mode='ns'):
        self.main_app.set_collapse_name(collapse_name)
        self.main_app.set_separate_mode(separate_mode)
        self.main_app.read_annotation_points()
        self.main_app.set_reference_points(reference_mode, ref_path,
                                           edge_dir, freq)
        self.main_app.make_dataset_stream(num_points, relative)

    def label(self, collapse_name, num_points=64, limit=32):
        self.main_app.set_collapse_name(collapse_name)
        self.main_app.read_dataset()
        self.main_app.make_label(num_points, limit)

    def clip(self, file_name):
        points = pd.read_csv('./rawdata/' + file_name, header=None)
        points = from_numpy_to_pcd(points.values)
        crop_geometries([points])


class MainApp():
    def __init__(self):
        self.config = configparser.ConfigParser()
        self.config.read('config.ini', encoding='utf-8')

    def set_variation_names(self, before_name, after_name):
        self.before_name = before_name
        self.after_name = after_name

    def set_collapse_name(self, collapse_name):
        self.collapse_name = collapse_name

    def set_separate_mode(self, separate_mode):
        self.separate_mode = separate_mode

    def import_raw_pointclouds(self):
        tmp_before_path = self.config['DEFAULT']['RawData_Path'] + \
            '/' + self.before_name + '.csv'
        tmp_after_path = self.config['DEFAULT']['RawData_Path'] + \
            '/' + self.after_name + '.csv'
        self.before_raw_pcd = read_csv_to_pcd(tmp_before_path)
        self.before_raw_pcd.paint_uniform_color(
            json.loads(self.config['DEFAULT']['YELLOW']))
        self.after_raw_pcd = read_csv_to_pcd(tmp_after_path)
        self.after_raw_pcd.paint_uniform_color(
            json.loads(self.config['DEFAULT']['BLUE']))

    def make_tmp_directry(self):
        os.makedirs('tmp/' + self.before_name, exist_ok=True)
        os.makedirs('tmp/' + self.after_name, exist_ok=True)

    def draw_variation(self):
        draw_geometries([self.before_raw_pcd, self.after_raw_pcd])

    def crop_loop(self):
        end_flag = False
        while not end_flag:
            print('input command')
            command = input()
            if command == 'c':
                crop_geometries([self.before_raw_pcd], window_name='Before')
                tmp_before_pcds = self.labeling_pcd(
                    self.before_name, self.before_raw_pcd)
                draw_geometries(tmp_before_pcds, window_name='Before')
                crop_geometries([self.after_raw_pcd], window_name='After')
                tmp_after_pcds = self.labeling_pcd(
                    self.after_name, self.after_raw_pcd)
                draw_geometries(tmp_after_pcds, window_name='After')
            elif command == 'q':
                end_flag = True
                tmp_before_pcds = self.labeling_pcd(
                    self.before_name, self.before_raw_pcd)
                tmp_after_pcds = self.labeling_pcd(
                    self.after_name, self.after_raw_pcd)
                self.annotation_export(tmp_before_pcds, tmp_after_pcds)
                print('quit')

    def annotation_export(self, before_pcds, after_pcds):
        print('input collapse name')
        self.collapse_name = input()
        os.makedirs('./intermediate/' + self.collapse_name, exist_ok=True)
        config = {
            'name': self.collapse_name,
            'before': self.before_name,
            'after': self.after_name
        }
        with open('./intermediate/' + self.collapse_name + '/config.json',
                  mode='wt', encoding='utf-8') as file:
            json.dump(config, file, ensure_ascii=False, indent=2)
        o3d.io.write_point_cloud(
            './intermediate/' + self.collapse_name + '/before_negative.ply',
            before_pcds[0]
        )
        o3d.io.write_point_cloud(
            './intermediate/' + self.collapse_name + '/before_positive.ply',
            before_pcds[1]
        )
        o3d.io.write_point_cloud(
            './intermediate/' + self.collapse_name + '/after_negative.ply',
            after_pcds[0]
        )
        o3d.io.write_point_cloud(
            './intermediate/' + self.collapse_name + '/after_positive.ply',
            after_pcds[1]
        )

    def registrate_variation(self):
        manual_registration([self.before_raw_pcd], [self.after_raw_pcd])

    def read_annotation_points(self):
        self.before_negative = o3d.io.read_point_cloud(
            './intermediate/' + self.collapse_name + '/before_negative.ply'
        )
        self.before_positive = o3d.io.read_point_cloud(
            './intermediate/' + self.collapse_name + '/before_positive.ply'
        )
        self.after_negative = o3d.io.read_point_cloud(
            './intermediate/' + self.collapse_name + '/after_negative.ply'
        )
        self.after_positive = o3d.io.read_point_cloud(
            './intermediate/' + self.collapse_name + '/after_positive.ply'
        )
        self.before_negative.paint_uniform_color(
            json.loads(self.config['DEFAULT']['YELLOW']))
        self.before_positive.paint_uniform_color(
            json.loads(self.config['DEFAULT']['BLUE']))
        self.after_negative.paint_uniform_color(
            json.loads(self.config['DEFAULT']['YELLOW']))
        self.after_positive.paint_uniform_color(
            json.loads(self.config['DEFAULT']['BLUE']))

    def import_directory_pointclouds(self, dir):
        files = glob.glob('./tmp/' + dir + '/*.ply')
        pcds = []
        for file in files:
            pcds.append(o3d.io.read_point_cloud(file))
        return pcds

    def remove_duplicate(self, dir):
        column_names = ['x', 'y', 'z']
        tmp_pcds = self.import_directory_pointclouds(dir)
        if len(tmp_pcds) == 0:
            return pd.DataFrame(None, columns=column_names)
        point_stack = np.concatenate([tmp_pcd.points for tmp_pcd in tmp_pcds])
        point_df = pd.DataFrame(point_stack, columns=column_names)
        unique_points = point_df[~point_df.duplicated()]
        return unique_points

    def labeling_pcd(self, dir, base_pcd):
        column_names = ['x', 'y', 'z']
        tmp_all_points_df = pd.DataFrame(
            base_pcd.points, columns=column_names)
        positive_point_df = self.remove_duplicate(dir)
        tmp_df = pd.concat([tmp_all_points_df, positive_point_df])
        negative_point_df = tmp_df[~tmp_df.duplicated(keep=False)]
        labeled_pcds = [
            from_numpy_to_pcd(negative_point_df.values),
            from_numpy_to_pcd(positive_point_df.values)
        ]
        labeled_pcds[0].paint_uniform_color(
            json.loads(self.config['DEFAULT']['YELLOW']))
        labeled_pcds[1].paint_uniform_color(
            json.loads(self.config['DEFAULT']['BLUE']))
        return labeled_pcds

    def set_reference_points(self, mode,
                             ref_path=None, edge_dir=None, freq=64):
        if mode == 'r':  # random pick
            self.get_reference_points_randomly(freq)
        elif mode == 'e':
            self.get_reference_points_within_edge(freq, edge_dir)
        elif mode == 'f':
            self.get_reference_points_from_csv(ref_path)

    def get_reference_points_randomly(self, freq):
        before = np.concatenate(
                    [self.before_negative.points, self.before_positive.points])
        reference_points_count = int(len(before) / freq)
        idx = np.random.randint(len(before), size=reference_points_count)
        self.reference_points = before[idx, :]

    def get_reference_points_within_edge(self, freq, edge_dir):
        files = glob.glob('./edge/' + edge_dir + '/*.ply')
        tmp_points = np.concatenate(
            [o3d.io.read_point_cloud(file_name).points for file_name in files])
        reference_points_count = int(len(tmp_points) / freq)
        idx = np.random.randint(len(tmp_points), size=reference_points_count)
        self.reference_points = tmp_points[idx, :]

    def get_reference_points_from_csv(self, file_path):
        point_df = pd.read_csv(file_path)
        self.reference_points = point_df.loc[:, ['x', 'y', 'z']].values

    def import_reference_points(self, n=64):
        end_flag = False
        while not end_flag:
            print('input command')
            command = input()
            if command == 'f':  # import csv
                self.get_reference_points_from_csv(
                    './intermediate/' + self.collapse_name + '/reference_points.csv')
                end_flag = True
            elif command == 'p':  # pick points
                self.reference_points = pick_points(
                    [self.before_negative, self.before_positive])
                end_flag = True
            elif command == 'e':  # random pick (within edge)
                print('範囲点群ディレクトリ名を入力')
                dir = input()
                self.get_reference_points_within_edge(n, dir)
                end_flag = True
            elif command == 'r':  # random pick
                self.get_random_reference_points(n)
                end_flag = True
        print('imported')

    def manual_registration(self):
        _, trans_matrix = manual_registration(
            self.after_raw_pcd, self.before_raw_pcd)

    def read_dataset(self):
        self.before_points = pd.read_csv(
            './dataset/' + self.collapse_name + '/before.csv')
        self.after_points = pd.read_csv(
            './dataset/' + self.collapse_name + '/after.csv')

    def make_label(self, n=64, limit=16):
        labels = [self.count_label(i, n=n, limit=limit)
                  for i in range(0, len(self.before_points), n)]
        label_df = pd.Series(labels, name='label')
        label_df.to_csv('./dataset/' + self.collapse_name + '/labels.csv')

    def count_label(self, start, *, n=64, limit=32):
        before_labels = self.before_points[start:start+n]['label'].values
        after_labels = self.after_points[start:start+n]['label'].values
        return 1 if sum([1 for i in range(n) if before_labels[i] or after_labels[i]]) > limit else 0

    def make_dataset_stream(self, num_points=64, relative=True):
        os.makedirs('./dataset/' + self.collapse_name, exist_ok=True)
        tmp_before_negative = pd.DataFrame(
            self.before_negative.points, columns=['x', 'y', 'z'])
        tmp_before_negative['label'] = 0
        tmp_before_positive = pd.DataFrame(
            self.before_positive.points, columns=['x', 'y', 'z'])
        tmp_before_positive['label'] = 1
        tmp_before_points_df = pd.concat([
            tmp_before_negative, tmp_before_positive
        ])
        tmp_after_negative = pd.DataFrame(
            self.after_negative.points, columns=['x', 'y', 'z'])
        tmp_after_negative['label'] = 0
        tmp_after_positive = pd.DataFrame(
            self.after_positive.points, columns=['x', 'y', 'z'])
        tmp_after_positive['label'] = 1
        tmp_after_points_df = pd.concat([
            tmp_after_negative, tmp_after_positive
        ])
        step = 0
        for reference_point in self.reference_points:
            point_separator = PointSeparator(num_points, self.separate_mode)
            before_near_points = point_separator.get_near_points(
                tmp_before_points_df,
                reference_point[0],
                reference_point[1],
                reference_point[2]
            )
            after_near_points = point_separator.get_near_points(
                tmp_after_points_df,
                reference_point[0],
                reference_point[1],
                reference_point[2]
            )
            if step == 0:
                before_near_points.to_csv(
                    './dataset/' + self.collapse_name + '/before.csv')
                after_near_points.to_csv(
                    './dataset/' + self.collapse_name + '/after.csv')
            else:
                before_near_points.to_csv(
                    './dataset/' + self.collapse_name + '/before.csv',
                    mode='a', header=False)
                after_near_points.to_csv(
                    './dataset/' + self.collapse_name + '/after.csv',
                    mode='a', header=False)
            if step % 100 == 0:
                print('{}/{}'.format(step, len(self.reference_points)))
            step += 1
