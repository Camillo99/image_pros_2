import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Define nodes
    node1 = launch_ros.actions.Node(
        package='image_pros_2',
        executable='circle_2D_topic.py',
        name='processing_2D',
        output='screen',
        emulate_tty=True
    )

    node2 = launch_ros.actions.Node(
        package='image_pros_2',
        executable='pc_topic',
        name='processing_3D',
        output='screen',
        emulate_tty=True
    )

    node3 = launch_ros.actions.Node(
        package='image_pros_2',
        executable='new_hole_detect.py',
        name='reprojection_process',
        output='screen',
        emulate_tty=True
    )

    node4 = launch_ros.actions.Node(
        package='image_pros_2',
        executable='call_capture_simple.py',
        name='call_capture',
        output='screen',
        emulate_tty=True
    )  


    
    #zivid node, following the default zivid_standalone.launch.py launcher from zivid_camera package

    # settings_path = PathJoinSubstitution(
    #     [FindPackageShare("zivid_camera"), "config", "zivid_camera_settings.yml"]
    # )
    # settings2d_path = PathJoinSubstitution(
    #     [FindPackageShare("zivid_camera"), "config", "zivid_camera_settings2d.yml"]
    # )
    settings_path = '/home/camillo/Documents/zivid_config03.yml'
    settings2d_path ='/home/camillo/Documents/zivid_config_2D_03.yml'

    zivid_camera_standalone = launch_ros.actions.Node(
        package="zivid_camera",
        executable="zivid_camera_standalone",
        name="camera",
        namespace="zivid",
        output="screen",
        parameters=[
            {"frame_id": "zivid_camera_frame"},
            {"settings_path": settings_path},
            {"settings2d_path": settings2d_path},
            {"file_camera_path": ""},
            {"update_firmware_automatically": False},
            {"use_latched_publisher_for_points_xyz": True},
        ],
    )
    
    # Create launch description
    ld = launch.LaunchDescription()

    # Add nodes to the launch description
    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
    """comment next line to do not run zivid camera node"""
    ld.add_action(zivid_camera_standalone)  
    # ld.add_action(node4)

    return ld
