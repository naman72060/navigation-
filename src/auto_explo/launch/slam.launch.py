import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription #type:ignore
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution #type:ignore
from launch_ros.actions import Node #type:ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource #type:ignore
from ament_index_python.packages import get_package_share_directory, get_package_share_directory #type:ignore


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # ---- Declare Arguments ----
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_localization.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # ---- Slam Toolbox Node ----
    start_localization_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # ---- Nav2 Launch ----
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')]
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ---- Final LaunchDescription ----
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_localization_slam_toolbox_node)
    ld.add_action(nav2_launch)

    return ld
