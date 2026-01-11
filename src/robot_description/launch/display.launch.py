from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os

def generate_launch_description():
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

    # GUI
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

    return LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node
    ])
