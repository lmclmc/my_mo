from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare(package='magic_vis_ros2').find('magic_vis_ros2')
    
    # Get RViz config path
    rviz_config_path = PathJoinSubstitution([pkg_share, 'magic_humanoid.rviz'])
    
    # Get URDF path as direct string
    urdf_file_path = os.path.join(pkg_share, '', 'p5_humanoid.urdf')
    
    # Robot visualizer node
    robot_vis_node = Node(
        package='magic_vis_ros2',
        executable='robot_vis',
        name='robot_urdf_visualizer',
        output='screen',
        parameters=[{
            'urdf_file_path': urdf_file_path,
            'urdf_binary_mode': True,
            'urdf_throw_on_error': True
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_magic',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add nodes
    ld.add_action(robot_vis_node)
    ld.add_action(rviz_node)
    
    return ld