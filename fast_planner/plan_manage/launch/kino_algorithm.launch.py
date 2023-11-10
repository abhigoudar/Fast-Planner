from launch_ros.actions import Node, SetRemap
from launch.launch_description import LaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #
    launch_description = [
        SetRemap(src='/odom_world', dst='/state_ukf/odom'),
        SetRemap(src='/sdf/odom', dst='/state_ukf/odom'),
        SetRemap(src='/sdf/depth', dst='/pcl_render_node/depth'),
        SetRemap(src='/sdf/cloud', dst='/pcl_render_node/cloud'),
    ]
    #
    launch_description.append(
        DeclareLaunchArgument(
            name='fast_planner_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('plan_manage'), 'config/fast_planner_config.yaml'
            ])
    ))
    #
    #
    launch_description.append(
        Node(
            name="fast_planner_node",
            package="plan_manage",
            executable="fast_planner_node",
            parameters=[LaunchConfiguration('fast_planner_config_file')],
            output="screen",
            emulate_tty=True,
        )
    )

    return LaunchDescription(launch_description)