# INTEL CONFIDENTIAL
# Copyright 2022 Intel Corporation.
# This software and the related documents are Intel copyrighted materials, and
# your use of them is governed by the express license under which they were
# provided to you ("License"). Unless the License provides otherwise, you may
# not use, modify, copy, publish, distribute, disclose or transmit this
# software or the related documents without Intel prior written permission.
# This software and the related documents are provided as is, with no express
# or implied warranties, other than those that are expressly stated in the
# License.


import os
import launch
import launch.actions
import launch.substitutions
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

low_res_config = {
        'depth_module.profile':'848,480,30',
        'rgb_camera.profile':'848,480,30',
        'enable_infra1': 'true',
        'align_depth.enable':'true'
        }

def generate_launch_description():
    # ================== Camera =========================
    tf_node = launch_ros.actions.Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           name='camera_tf',
           arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
           parameters=[{'use_sim_time': False}]
   )
    # =================== Object Detection ===================
    od_launch = launch_ros.actions.Node(
            #node initialization as before
            name='object_detection',package='object_detection', executable='object_detection_node', output='screen'
        )

    realsense2_dir = get_package_share_directory('realsense2_camera')
    realsense_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            realsense2_dir + '/launch/rs_launch.py'),
                launch_arguments=low_res_config.items()
    )

    ovms_launch = launch_ros.actions.Node(
            #node initialization as before
            name='remote_inference',package='remote_inference', executable='remote_inference', output='screen',
            parameters=[
                {"remote_hostname": "put-server-hostname-here:3335"},
                {"model_name": "ssd_mobilenet_v2_coco"},
                {"model_version": 1}
                ]
        )

    return launch.LaunchDescription([
        tf_node,
        realsense_launch,
        od_launch,
        ovms_launch
    ])
