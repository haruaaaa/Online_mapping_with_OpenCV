from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os

def generate_launch_description():

    param_file = '/home/src/lidar_vibes/config/slam_params.yaml' #сменить директорию

    pkg_share = FindPackageShare('robot_description').find('robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'lego_robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # # GUI
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # RViz
    rviz_node = TimerAction(
        period=2.0,  # достаточно 2 секунд
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'lego_config.rviz')],
            name='rviz2',
        )]
    )

    client_node = Node(
        package='driver',
        executable='client',
        name='driver_client',
        output='screen'
    )

    laser_node = Node(
        package='lidar_vibes',
        executable='laser_pusk',
        name='lidar_vibes',
        output='screen'
    )

    moving = Node(
        package='exploring',
        executable='explore_cv',
        name='explore_cv',
        output='screen'
    )

    slam_launch= PathJoinSubstitution([
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py',
        
    ])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "slam_params_file": param_file,
            "use_sim_time": "true"
        }.items(),
    )
    # сменить директорию
    nav2 = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'params_file:=/home/haruaha/testo_ws/src/lidar_vibes/config/nav2_params.yaml'],
        output='screen'
    )
    
    nav2_1 = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'localization_launch.py', 'params_file:=/home/haruaha/testo_ws/src/lidar_vibes/config/nav2_params.yaml'],
        output='screen'
    )

    return LaunchDescription([
        
        rsp_node,
        jsp_node,
        client_node,
        laser_node,
        slam,
        rviz_node,
        nav2,
        nav2_1,
        moving
    ])