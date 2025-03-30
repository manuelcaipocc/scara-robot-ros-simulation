import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Encuentra el archivo .xacro
    scara_pkg_path = get_package_share_directory('scara_robot')
    xacro_file = os.path.join(scara_pkg_path, 'urdf', 'scara.xacro')
    moveit_pkg_path = get_package_share_directory('scara_robot_movelt2')

    # Archivos
    xacro_file = os.path.join(scara_pkg_path, 'urdf', 'scara.xacro')
    rviz_config_file = os.path.join(moveit_pkg_path, 'config', 'moveit.rviz')
    move_group_launch_file = os.path.join(moveit_pkg_path, 'launch', 'move_group.launch.py')

    # Procesa el archivo xacro a URDF
    robot_description_config = xacro.process_file(xacro_file).toxml()
    print("DEBUG robot_description length:", len(robot_description_config))

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(scara_pkg_path, 'rviz', 'scara.rviz')]
        )
    ])
