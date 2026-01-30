from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declare_robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='p5',
        description='Robot model name'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare(package='magic_vis_ros2').find('magic_vis_ros2')

    # Include robot specific launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PythonExpression([
                '"', pkg_share, '/launch/', LaunchConfiguration('robot'), '.launch"'
            ])
        )
    )
    print(pkg_share)
    print(str(LaunchConfiguration('robot')))
    
    # RViz node
    rviz_config_path = PathJoinSubstitution([pkg_share, '', 'magic_humanoid.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_magic',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add declared arguments
    ld.add_action(declare_robot_arg)
    
    # Add actions
    ld.add_action(robot_launch)
    ld.add_action(rviz_node)
    
    return ld