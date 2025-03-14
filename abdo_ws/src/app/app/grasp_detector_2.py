import os
import cv2
import sys
import yaml
import time
import torch
import json
import argparse
from PIL import Image
import numpy as np
import open3d as o3d
from tqdm import tqdm
from torch.utils.data import DataLoader
from graspnetAPI.graspnet_eval import GraspGroup, GraspNetEval
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.insert(0, ROOT_DIR)
from find_grasp.utils.data_utils import CameraInfo,create_point_cloud_from_depth_image

from find_grasp.models.graspnet import GraspNet, pred_grasp_decode
from find_grasp.dataset.graspnet_dataset import GraspNetDataset, minkowski_collate_fn, load_grasp_labels
from find_grasp.utils.collision_detector import ModelFreeCollisionDetector


from grasp_detector_interface import GraspDetectorInterface


parser = argparse.ArgumentParser()
# variable in shell
parser.add_argument('--infer', action='store_true', default=False)
parser.add_argument('--eval', action='store_true', default=False)
parser.add_argument('--log_dir', default=os.path.join(ROOT_DIR + '/find_grasp/logs'), required=False)
parser.add_argument('--collision_thresh', type=float, default=0.01,
                    help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--epoch_range', type=list, default=[79], help='epochs to infer&eval')
parser.add_argument('--split', default='test_seen', choices=['test', 'test_seen', 'test_similar', 'test_novel']) # corresp to evaluate func

# default in shell
parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during inference [default: 1]')
parser.add_argument('--seed_feat_dim', default=256, type=int, help='Point wise feature dim')
parser.add_argument('--camera', default='realsense', help='Camera split [realsense/kinect]')
parser.add_argument('--num_point', type=int, default=25000, help='Point Number [default: 15000]')
parser.add_argument('--voxel_size', type=float, default=0.005, help='Voxel Size for sparse convolution')
parser.add_argument('--voxel_size_cd', type=float, default=0.01, help='Voxel Size for collision detection')
cfgs = parser.parse_args()


# load model config
with open(os.path.join(ROOT_DIR + '/find_grasp/models/model_config.yaml'), 'r') as f:
    model_config = yaml.load(f, Loader=yaml.FullLoader)

with open('/home/artc/UR_Grasp_Driver/yc_ws/src/my_ur_driver/Realsense Calibration Data/results.json') as f:
    fil = json.load(f)
    labels = fil["Intrinsics_labels"]
    values = fil["Intrinsics_values"]
    intrinsics = dict(zip(labels, values))
    fx = intrinsics.get("fx")
    fy = intrinsics.get("fy")
    cx = intrinsics.get("cx")
    cy = intrinsics.get("cy")
    width = intrinsics.get("width")
    height = intrinsics.get("height")
    print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}, width: {width}, height: {height}")
    factor_depth = 1000
    camera = CameraInfo(width, height, fx, fy, cx, cy, factor_depth)



class GraspPoseInference(GraspDetectorInterface):
    def __init__(self, coordinator=None):
        """
        Initializes the GraspDetector with the given configurations.

        Args:
            checkpoint_path (str): Path to the AnyGrasp model checkpoint.
            max_gripper_width (float): Maximum gripper width in meters (default: 0.1m).
            gripper_height (float): Gripper height in meters (default: 0.03m).
            top_down_grasp (bool): Whether to output top-down grasps (default: False).
            debug (bool): Enable debug mode with visualizations (default: True).
        """

        self.coordinator = coordinator

        self.cfgs = parser.parse_args()
        #self.cfgs.max_gripper_width = max(0, min(0.1, self.cfgs.max_gripper_width))

        # Camera intrinsics
        # self.fx = 642.4262770792711
        # self.fy = 642.3461744750167
        # self.cx = 647.5434733474444
        # self.cy = 373.3602344467871
        # self.scale = 1000.0
        self.camera = camera

        self.xmin, self.xmax = -0.25, 0.20
        self.ymin, self.ymax = -.2, 0.4
        self.zmin, self.zmax = -0.1, 1.0

        self.pointcloud = None
        self.color = None

        self.lims = [self.xmin, self.xmax, self.ymin, self.ymax ,self.zmin, self.zmax]
        self.model_config = model_config

        self.selected_grasp = None



    def get_my_data(self, color_image, depth_image):

        color = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

        depth = depth_image
        
        cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
        # color = color / 255
        cloud = cloud.reshape(-1, 3) 
        # xmin, ymin, zmin = -0.29 , -0.40 , 0
        # xmax, ymax, zmax = 0.50, 0.40, 1.0

        xmin, ymin, zmin = -0.25,-0.2 , -0.1
        xmax, ymax, zmax = 0.20, 0.4, 1.0

        mask_x = ((cloud[:, 0] > xmin) & (cloud[:, 0] < xmax))
        mask_y = ((cloud[:, 1] > ymin) & (cloud[:, 1] < ymax))
        mask_z = ((cloud[:, 2] > zmin) & (cloud[:, 2] < zmax))
        workspace_mask = (mask_x & mask_y & mask_z)
        #depth_mask = (depth > 0)
        mask = workspace_mask
        # print("Shape of cloud:", cloud.shape)
        # print("Shape of mask:", mask.shape)
        cloud_masked = cloud[mask]
        # scloud_masked= cloud_masked.reshape(720, 1280, 3)
        mask = mask.reshape(720,1280)
        color_masked = color[mask]
        if len(cloud_masked) >= cfgs.num_point:
            idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
        else:
            idxs1 = np.arange(len(cloud_masked))
            idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point - len(cloud_masked), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]
        ret_dict = {'point_clouds': cloud_sampled.astype(np.float32),
                    'coors': cloud_sampled.astype(np.float32) / cfgs.voxel_size,
                    'feats': np.ones_like(cloud_sampled).astype(np.float32),
                    'color': color_sampled.astype(np.float32),
                    }
        return ret_dict



    def detect_grasps(self, color_image, depth_image):
        sample_data = self.get_my_data(color_image, depth_image)
        self.pointcloud = sample_data['point_clouds']
        self.color = sample_data['color']
        net = GraspNet(self.model_config, is_training=False)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        net.to(device)
        epoch = 29

        checkpoint_path = os.path.join(cfgs.log_dir, 'epoch_{}.tar'.format(epoch))
        checkpoint = torch.load(checkpoint_path)
        net.load_state_dict(checkpoint['model_state_dict'])
        sample_data = minkowski_collate_fn([sample_data])
        for key in sample_data:
            if 'list' in key:
                for i in range(len(sample_data[key])):
                    for j in range(len(sample_data[key][i])):
                        sample_data[key][i][j] = sample_data[key][i][j].to(device)
            else:
                sample_data[key] = sample_data[key].to(device)

        net.eval()
        with torch.no_grad():
            source_end_points = net(sample_data)
            grasp_preds = pred_grasp_decode(source_end_points)  

        preds = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(preds)
        gg = gg.sort_by_score()

        indices_to_remove = []
        # xmin, ymin, zmin = -0.29 , -0.40 , 0
        # xmax, ymax, zmax = 0.50, 0.40, 1.0
        for i, grasp in enumerate(gg):
            if (grasp.translation[0] <= -0.2 or grasp.translation[0] >= 0.35) or \
                (grasp.translation[1] <= -0.2 or grasp.translation[1] >= 0.1):
                indices_to_remove.append(i)
        gg = gg.remove(indices_to_remove)
        gg_pick = gg[0]

        grippers = gg.to_open3d_geometry_list()
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(self.pointcloud.astype(np.float32))
        cloud.colors = o3d.utility.Vector3dVector(np.asarray(self.color, dtype=np.float32))
        o3d.visualization.draw_geometries([cloud, *grippers[:5]])

        #self.visualize(gg)

        self.selected_grasp = gg_pick

        return self.selected_grasp

    def visualize(self, gg):
        gg = self.detect_grasps(29)
        grippers = gg.to_open3d_geometry_list()
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(self.pointcloud.astype(np.float32))
        cloud.colors = o3d.utility.Vector3dVector(np.asarray(self.color, dtype=np.float32))
        o3d.visualization.draw_geometries([cloud, *grippers[:5]])
