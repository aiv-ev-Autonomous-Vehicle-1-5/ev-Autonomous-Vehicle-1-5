"""카메라 파이프라인 통합 launch: usb_cam → bev_node → yolo_db_seg."""

import os
import sys
from pathlib import Path

import numpy as np
import yaml
from tf_transformations import quaternion_from_matrix

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ── usb_cam 패키지의 camera_config 사용 ──
USB_CAM_DIR = get_package_share_directory('usb_cam')
sys.path.append(os.path.join(USB_CAM_DIR, 'launch'))
from camera_config import CameraConfig  # noqa: E402

TF_CONFIG_DIR = Path(USB_CAM_DIR, 'config', 'TFConfig')

CAMERAS = [
    CameraConfig(
        name='camera1',
        param_path=Path(USB_CAM_DIR, 'config', 'params_1.yaml'),
    ),
]


def load_tf_config(camera_name: str):
    """TF 설정 파일에서 변환 매개변수를 로드한다."""
    tf_config_path = Path(TF_CONFIG_DIR, f'{camera_name}_tf.yaml')
    if not tf_config_path.exists():
        return None

    with open(tf_config_path, 'r') as f:
        config = yaml.safe_load(f)

    rot = config['rotation_matrix']
    rot_matrix = np.array([
        [rot['r11'], rot['r12'], rot['r13'], 0],
        [rot['r21'], rot['r22'], rot['r23'], 0],
        [rot['r31'], rot['r32'], rot['r33'], 0],
        [0, 0, 0, 1],
    ])
    quat = quaternion_from_matrix(rot_matrix)
    t = config['translation']
    return {
        'parent_frame': config['parent_frame'],
        'child_frame': config['child_frame'],
        'x': t['x'], 'y': t['y'], 'z': t['z'],
        'qx': quat[0], 'qy': quat[1], 'qz': quat[2], 'qw': quat[3],
    }


def generate_launch_description():
    ld = LaunchDescription()

    # ── 1) usb_cam 노드 + camera TF ──
    for camera in CAMERAS:
        ld.add_action(Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings,
            output='screen',
        ))

        tf_config = load_tf_config(camera.name)
        if tf_config:
            ld.add_action(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'tf_{camera.name}_from_velodyne',
                arguments=[
                    str(tf_config['x']), str(tf_config['y']), str(tf_config['z']),
                    str(tf_config['qx']), str(tf_config['qy']),
                    str(tf_config['qz']), str(tf_config['qw']),
                    tf_config['parent_frame'], tf_config['child_frame'],
                ],
                output='screen',
            ))

    # ── 2) BEV 변환 노드 ──
    ld.add_action(Node(
        package='lane_seg',
        executable='bev_node',
        name='bev_lut_node',
        output='screen',
    ))

    # ── 3) YOLO DBSCAN 차선 세그멘테이션 노드 ──
    ld.add_action(Node(
        package='lane_seg',
        executable='yolo_db_seg',
        name='yolo_db_seg_node',
        output='screen',
    ))

    return ld
