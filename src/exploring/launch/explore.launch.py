import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    exploring_pkg = get_package_share_directory('exploring')
    slam_pkg = get_package_share_directory('slam_toolbox')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    default_slam_params = os.path.join(exploring_pkg, 'config', 'slam_params.yaml')
    default_nav2_params = os.path.join(exploring_pkg, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(exploring_pkg, 'rviz', 'exploration.rviz')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (set false on physical robot)'
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame of the robot (base_link or base_footprint)'
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='livox',
        description='TF frame of the Livox MID-360 LiDAR'
    )
    livox_topic_arg = DeclareLaunchArgument(
        'livox_topic',
        default_value='/livox/lidar',
        description='Livox MID-360 PointCloud2 topic'
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='2D LaserScan topic for SLAM and Costmaps'
    )
    convert_livox_arg = DeclareLaunchArgument(
        'convert_livox',
        default_value='true',
        description='Convert Livox MID-360 PointCloud2 to filtered 2D LaserScan'
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish static transform base_link -> livox (0, 0, 0.2m)'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 for visualization (default: false)'
    )
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to launch SLAM Toolbox'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Whether to launch Nav2 navigation stack'
    )
    explorer_arg = DeclareLaunchArgument(
        'explorer',
        default_value='true',
        description='Whether to launch OpenCV Frontier Explorer node'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to SLAM parameters YAML file'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to Nav2 parameters YAML file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    livox_topic = LaunchConfiguration('livox_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    params_file = LaunchConfiguration('params_file')

    # 1. Static TF (base_link -> livox) if not already published by robot base
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'livox'],
        condition=IfCondition(LaunchConfiguration('publish_tf'))
    )

    # 2. Livox MID-360 PointCloud2 to 2D LaserScan with ground/ceiling slicing
    livox_converter_node = Node(
        package='exploring',
        executable='livox_to_laserscan',
        name='livox_to_laserscan',
        output='screen',
        parameters=[{
            'cloud_topic': livox_topic,
            'scan_topic': scan_topic,
            'target_frame': lidar_frame,
            'min_height': -0.12,
            'max_height': 0.35,
            'min_range': 0.16,
            'max_range': 8.0,
            'num_rays': 720
        }],
        condition=IfCondition(LaunchConfiguration('convert_livox'))
    )

    # 3. SLAM Toolbox (starts immediately)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # 4. Nav2 Navigation Stack (starts after 3.0s to allow SLAM map & TF to initialize)
    nav2_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': 'true'
                }.items(),
                condition=IfCondition(LaunchConfiguration('nav2'))
            )
        ]
    )

    # 5. Frontier Explorer Node (OpenCV) (starts after 5.5s to let Nav2 action server activate)
    explorer_node = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='exploring',
                executable='explore_cv',
                name='explore_cv',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'base_frame': base_frame,
                    'map_frame': 'map',
                    'map_topic': '/map',
                    'goal_topic': '/goal_pose',
                    'min_cluster_size': 5,
                    'min_obstacle_clearance': 0.25,
                    'min_fallback_clearance': 0.18,
                    'goal_tolerance': 0.30,
                    'goal_timeout_sec': 40.0,
                    'stuck_timeout_sec': 12.0
                }],
                condition=IfCondition(LaunchConfiguration('explorer'))
            )
        ]
    )

    # 6. Optional RViz2 Visualization (default: disabled)
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', default_rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz'))
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        base_frame_arg,
        lidar_frame_arg,
        livox_topic_arg,
        scan_topic_arg,
        convert_livox_arg,
        publish_tf_arg,
        rviz_arg,
        slam_arg,
        nav2_arg,
        explorer_arg,
        slam_params_arg,
        nav2_params_arg,
        static_tf_node,
        livox_converter_node,
        slam_launch,
        nav2_launch,
        explorer_node,
        rviz_node
    ])
