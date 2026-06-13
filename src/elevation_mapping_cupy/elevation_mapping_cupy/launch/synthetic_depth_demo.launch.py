import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "elevation_mapping_cupy"
    share_dir = get_package_share_directory(package_name)

    core_param_path = os.path.join(share_dir, "config", "core", "core_param.yaml")
    setup_param_path = os.path.join(
        share_dir, "config", "setups", "synthetic", "synthetic_depth.yaml"
    )
    rviz_config_default = PathJoinSubstitution([share_dir, "rviz", "synthetic_demo.rviz"])

    if not os.path.exists(core_param_path):
        raise FileNotFoundError(f"Missing core params: {core_param_path}")
    if not os.path.exists(setup_param_path):
        raise FileNotFoundError(f"Missing setup params: {setup_param_path}")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use /clock if true"
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Launch RViz2"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_default,
        description="Path to RViz config",
    )
    publish_map_to_odom_arg = DeclareLaunchArgument(
        "publish_map_to_odom",
        default_value="true",
        description=(
            "Publish a fake static map->odom transform. Set false when AMCL or "
            "another localization source already publishes map->odom."
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    publish_map_to_odom = LaunchConfiguration("publish_map_to_odom")

    synthetic_pub = Node(
        package=package_name,
        executable="synthetic_pointcloud_tf_publisher.py",
        name="synthetic_pointcloud_tf_publisher",
        output="screen",
        parameters=[
            {
                # Publish the moving robot pose as odom -> base_link.
                # A static map -> odom below gives Nav2 the standard TF chain:
                # map -> odom -> base_link.
                "map_frame": "odom",
                "base_frame": "base_link",
                "pointcloud_topic": "/camera/depth/points",
                "publish_rate_hz": 10.0,
            }
        ],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
        condition=IfCondition(publish_map_to_odom),
    )

    elevation_mapping_node = Node(
        package=package_name,
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            core_param_path,
            setup_param_path,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            launch_rviz_arg,
            rviz_config_arg,
            publish_map_to_odom_arg,
            map_to_odom_tf,
            synthetic_pub,
            elevation_mapping_node,
            rviz_node,
        ]
    )

