from pandas.io.formats.format import return_docstring
from point_cloud.visualise import crop_geometries, draw_geometries
from point_cloud.conversion import from_numpy_to_pcd
from point_cloud.operations import manual_registration, get_near_points, pick_points
from common.csv import read_csv_to_pcd
import configparser
import json
import os
import glob
import open3d as o3d
import numpy as np
import pandas as pd


class MainApp():
    def __init__(self):
        print('main app!')
        self.config = configparser.ConfigParser()
        self.config.read('config.ini', encoding='utf-8')

    def run_annotation(self):
        end_flag = False
        self.input_name()
        self.import_raw_pointclouds()
        draw_geometries([self.before_raw_pcd, self.after_raw_pcd])
        os.makedirs('tmp/' + self.before_name, exist_ok=True)
        os.makedirs('tmp/' + self.after_name, exist_ok=True)
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

    def run_registration(self):
        self.input_name()
        self.import_raw_pointclouds()
        manual_registration(self.before_raw_pcd, self.after_raw_pcd)

    def run_split(self):
        self.collapse_name = input()
        self.read_annotation_points()
        self.import_reference_points()
        self.make_dataset()

    def input_name(self):
        print('変化前形状名を入力')
        self.before_name = input()
        print('変化後形状名を入力')
        self.after_name = input()

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

    def import_reference_points(self, n=64):
        end_flag = False
        while not end_flag:
            print('input command')
            command = input()
            if command == 'f':  # import csv
                point_df = pd.read_csv(
                    './intermediate/' + self.collapse_name + '/reference_points.csv')
                self.reference_points = point_df.loc[:, ['x', 'y', 'z']].values
                end_flag = True
            elif command == 'p':  # pick points
                self.reference_points = pick_points(
                    [self.before_negative, self.before_positive])
                end_flag = True
            elif command == 'r':  # random pick
                before = np.concatenate(
                    [self.before_negative.points, self.before_positive.points])
                reference_points_count = int(len(before) / n)
                idx = np.random.randint(
                    len(before), size=reference_points_count)
                self.reference_points = before[idx, :]
                end_flag = True

    def manual_registration(self):
        _, trans_matrix = manual_registration(
            self.after_raw_pcd, self.before_raw_pcd)

    def make_dataset(self):
        tmp_before_negative = pd.DataFrame(
            self.before_negative.points, columns=['x', 'y', 'z'])
        tmp_before_negative['label'] = 0
        tmp_before_positive = pd.DataFrame(
            self.before_positive.points, columns=['x', 'y', 'z'])
        tmp_before_positive['label'] = 1
        tmp_before_points_df = pd.concat([
            tmp_before_negative, tmp_before_positive
        ])
        tmp_before_points_df.to_csv('test.csv')
        self.before_stack = pd.concat([get_near_points(
            tmp_before_points_df,
            reference_point[0],
            reference_point[1],
            reference_point[2]
        )
            for reference_point in self.reference_points])
        os.makedirs('./dataset/' + self.collapse_name, exist_ok=True)
        self.before_stack.to_csv(
            './dataset/' + self.collapse_name + '/before.csv')
        tmp_after_negative = pd.DataFrame(
            self.after_negative.points, columns=['x', 'y', 'z'])
        tmp_after_negative['label'] = 0
        tmp_after_positive = pd.DataFrame(
            self.after_positive.points, columns=['x', 'y', 'z'])
        tmp_after_positive['label'] = 1
        tmp_after_points_df = pd.concat([
            tmp_after_negative, tmp_after_positive
        ])
        self.after_stack = pd.concat([get_near_points(
            tmp_after_points_df,
            reference_point[0],
            reference_point[1],
            reference_point[2]
        )
            for reference_point in self.reference_points])
        self.before_stack.to_csv(
            './dataset/' + self.collapse_name + '/after.csv')

    def run_label_maker(self):
        print('input collapse name')
        self.collapse_name = input()
        self.read_dataset()
        self.make_label()
        self.count_label(0)

    def read_dataset(self):
        self.before_points = pd.read_csv(
            './dataset/' + self.collapse_name + '/before.csv')
        self.after_points = pd.read_csv(
            './dataset/' + self.collapse_name + '/after.csv')

    def make_label(self, n=64):
        labels = [self.count_label(i, n) for i in range(0, len(self.before_points), 64)]
        label_df = pd.Series(labels, name='label')
        label_df.to_csv('./dataset/' + self.collapse_name + '/labels.csv')

    def count_label(self, start, n=64, limit=32):
        before_labels = self.before_points[start:start+n]['label'].values
        after_labels = self.after_points[start:start+n]['label'].values
        return 1 if sum([1 for i in range(n) if before_labels[i] or after_labels[i]]) > limit else 0
