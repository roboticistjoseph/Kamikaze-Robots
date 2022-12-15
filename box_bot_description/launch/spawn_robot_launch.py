#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    urdf = os.path.join(get_package_share_directory('box_bot_description'), 'robot/', 'box_bot.urdf')
    assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)

    ld = LaunchDescription([
        #Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='box_bot_description', executable='spawn_box_bot.py', arguments=[urdf], output='screen'),
    ])
    return ld