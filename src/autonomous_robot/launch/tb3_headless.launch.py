from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ─────────────────────────────────────────────
    # Turtlebot3 modell
    # ─────────────────────────────────────────────
    turtlebot_model = "burger"

    gazebo_models_path = "/opt/ros/humble/share/turtlebot3_gazebo/models"
    gazebo_resource_path = "/opt/ros/humble/share/turtlebot3_gazebo"

    world_file = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "worlds",
        "turtlebot3_world.world",
    )

    # ─────────────────────────────────────────────
    # Gazebo headless (GUI nélkül)
    # ─────────────────────────────────────────────
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world_file,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # ─────────────────────────────────────────────
    # Robot State Publisher (URDF)
    # ─────────────────────────────────────────────
    robot_description_file = os.path.join(
        get_package_share_directory("turtlebot3_description"),
        "urdf",
        f"turtlebot3_{turtlebot_model}.urdf",
    )

    with open(robot_description_file, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ─────────────────────────────────────────────
    # Turtlebot spawn Gazebo-ba
    # ─────────────────────────────────────────────
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "turtlebot3_burger",
            "-file", os.path.join(gazebo_models_path, "turtlebot3_burger", "model.sdf"),
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.01",
        ],
        output="screen",
    )

    # ─────────────────────────────────────────────
    # FIX TF: base_footprint → base_scan
    # (LaserScan miatt kötelező)
    # ─────────────────────────────────────────────
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="basefootprint_to_basescan_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_scan"],
        output="screen",
    )

    # ─────────────────────────────────────────────
    # Controller node
    # ─────────────────────────────────────────────
    controller_node = Node(
        package="autonomous_robot",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[{"use_sim_time": True, "stop_distance": 0.5}],
    )

    # ─────────────────────────────────────────────
    # Min distance + collision warning node
    # ─────────────────────────────────────────────
    min_distance_node = Node(
        package="autonomous_robot",
        executable="min_distance_node",
        name="min_distance_node",
        output="screen",
        parameters=[{"use_sim_time": True, "stop_distance": 0.5}],
    )

    # ─────────────────────────────────────────────
    # /cmd_vel monitor (csak logolás)
    # ─────────────────────────────────────────────
    cmd_vel_monitor = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="autonomous_robot",
                executable="cmd_vel_monitor",
                name="cmd_vel_monitor",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # ─────────────────────────────────────────────
    # Launch description
    # ─────────────────────────────────────────────
    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", turtlebot_model),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_models_path),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", gazebo_resource_path),

        gazebo_server,
        robot_state_publisher,
        spawn_robot,

        static_tf,
        controller_node,
        min_distance_node,
        cmd_vel_monitor,
    ])

