import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='articubot_one' 

    rectify_left = Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_left_node',
          
            remappings=[
                ('image', '/cameraL/image_raw'),
                ('camera_info', '/cameraL/camera_info'),
                ('image_rect', '/cameraL/image_rect')
            ]
        )
    rectify_right = Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_right_node',
  
            remappings=[
                ('image', '/cameraR/image_raw'),
                ('camera_info', '/cameraR/camera_info'),
                ('image_rect', '/cameraR/image_rect')
            ]
        )

    sbs_stitcher = Node(
            package=package_name,
            executable='sbs_merger_node.py', # Asegúrate de que tenga permisos de ejecución
            name='sbs_stitcher',
            remappings=[
                ('left_image', '/cameraL/image_rect'),
                ('right_image', '/cameraR/image_rect'),
                ('output_image', '/vr/sbs_final')
            ]
        )

    
    return LaunchDescription([
            rectify_left,
            rectify_right,
            sbs_stitcher
    ])
