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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    object_detection_ssd = Node(
        package="object_detection",
        executable="object_detection_node",
        parameters=[
            {"device": "MYRIAD"}
        ]
    )

    ld.add_action(object_detection_ssd)
    return ld
