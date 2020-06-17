# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/liu/Multi_Robot_System/devel_isolated/hector_pose_estimation_core;/home/liu/Multi_Robot_System/devel_isolated/hector_map_server;/home/liu/Multi_Robot_System/devel_isolated/hector_geotiff_plugins;/home/liu/Multi_Robot_System/devel_isolated/hector_geotiff;/home/liu/Multi_Robot_System/devel_isolated/hector_nav_msgs;/home/liu/Multi_Robot_System/devel_isolated/hector_models;/home/liu/Multi_Robot_System/devel_isolated/hector_marker_drawing;/home/liu/Multi_Robot_System/devel_isolated/hector_mapping;/home/liu/Multi_Robot_System/devel_isolated/hector_compressed_map_transport;/home/liu/Multi_Robot_System/devel_isolated/hector_map_tools;/home/liu/Multi_Robot_System/devel_isolated/hector_localization;/home/liu/Multi_Robot_System/devel_isolated/hector_imu_tools;/home/liu/Multi_Robot_System/devel_isolated/hector_imu_attitude_to_tf;/home/liu/Multi_Robot_System/devel_isolated/hector_gazebo_worlds;/home/liu/Multi_Robot_System/devel_isolated/hector_gazebo_thermal_camera;/home/liu/Multi_Robot_System/devel_isolated/hector_gazebo_plugins;/home/liu/Multi_Robot_System/devel_isolated/hector_gazebo;/home/liu/Multi_Robot_System/devel_isolated/hector_components_description;/home/liu/Multi_Robot_System/devel_isolated/ee_command;/home/liu/Multi_Robot_System/devel_isolated/arm_training;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/liu/Multi_Robot_System/devel_isolated/hector_pose_estimation/env.sh')

output_filename = '/home/liu/Multi_Robot_System/build_isolated/hector_pose_estimation/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
