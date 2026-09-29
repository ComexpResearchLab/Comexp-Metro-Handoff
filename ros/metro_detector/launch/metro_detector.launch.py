"""Launch the metro obstacle detector (and optionally RViz 2).

  ros2 launch metro_detector metro_detector.launch.py
  ros2 launch metro_detector metro_detector.launch.py input_topics:="['/lidar_points']" verbose:=true
  ros2 launch metro_detector metro_detector.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('metro_detector')
    args = [
        DeclareLaunchArgument('input_topics', default_value="['/lidar_points', '/sensing/lidar/hesai128/pointcloud']",
                              description='PointCloud2 topics (one tracker per topic)'),
        DeclareLaunchArgument('alerts_topic', default_value='/metro/alerts'),
        DeclareLaunchArgument('obstacle_topic', default_value='/metro/obstacle'),
        DeclareLaunchArgument('distance_topic', default_value='/metro/obstacle_distance'),
        DeclareLaunchArgument('markers_topic', default_value='/metro/markers'),
        DeclareLaunchArgument('status_topic', default_value='/metro/status'),
        DeclareLaunchArgument('frame_id', default_value='', description="marker frame; '' = the cloud's own frame_id"),
        DeclareLaunchArgument('verbose', default_value='false', description='log every frame'),
        DeclareLaunchArgument('publish_markers', default_value='true'),
        DeclareLaunchArgument('qos_depth', default_value='100', description='DDS reader history (frames)'),
        DeclareLaunchArgument('max_queue', default_value='1', description='frames waiting per stage once locked (older ones dropped)'),
        DeclareLaunchArgument('warmup_queue', default_value='20', description='frames kept per stage while the odometry warms up (all of them)'),
        DeclareLaunchArgument('max_skip', default_value='0.5', description='s: a stage never drops frames beyond this stamp step'),
        DeclareLaunchArgument('use_stamps', default_value='true', description='header stamps to the engine (bridges skipped frames)'),
        DeclareLaunchArgument('rust_regrid', default_value='true', description='ring-less clouds regridded in the engine library'),
        DeclareLaunchArgument('qos_reliable', default_value='true', description='false for a best-effort publisher'),
        DeclareLaunchArgument('record_path', default_value='', description='JSONL per frame (offline checks)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    L = LaunchConfiguration
    node = Node(
        package='metro_detector', executable='metro_detector', name='metro_detector', output='screen',
        parameters=[{
            'input_topics': L('input_topics'),          # YAML list string -> string array
            'alerts_topic': L('alerts_topic'), 'obstacle_topic': L('obstacle_topic'),
            'distance_topic': L('distance_topic'), 'markers_topic': L('markers_topic'), 'status_topic': L('status_topic'),
            'frame_id': ParameterValue(L('frame_id'), value_type=str),
            'verbose': ParameterValue(L('verbose'), value_type=bool),
            'publish_markers': ParameterValue(L('publish_markers'), value_type=bool),
            'qos_depth': ParameterValue(L('qos_depth'), value_type=int),
            'max_queue': ParameterValue(L('max_queue'), value_type=int),
            'warmup_queue': ParameterValue(L('warmup_queue'), value_type=int),
            'max_skip': ParameterValue(L('max_skip'), value_type=float),
            'use_stamps': ParameterValue(L('use_stamps'), value_type=bool),
            'rust_regrid': ParameterValue(L('rust_regrid'), value_type=bool),
            'qos_reliable': ParameterValue(L('qos_reliable'), value_type=bool),
            'record_path': ParameterValue(L('record_path'), value_type=str),
            'use_sim_time': ParameterValue(L('use_sim_time'), value_type=bool),
        }],
    )
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                arguments=['-d', os.path.join(share, 'config', 'metro.rviz')], condition=IfCondition(L('rviz')))
    return LaunchDescription(args + [node, rviz])
