from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    world = "/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world"
    model_sdf = "/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
    urdf = "/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf"

    return LaunchDescription([
        # TurtleBot3 modell
        SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger"),

        # Gazebo resource/model path (kell a model://turtlebot3_world feloldásához)
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value="/opt/ros/humble/share/turtlebot3_gazebo/models"
        ),
        SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value="/opt/ros/humble/share/turtlebot3_gazebo"
        ),

        # Gazebo server indítása GUI nélkül + ROS pluginok
        ExecuteProcess(
            cmd=[
                "gzserver",
                world,
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
            ],
            output="screen"
        ),

        # robot_state_publisher (TF/RViz-hez) - URDF-ből betöltve
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    parameters=[
                        {"use_sim_time": True},
                        {"robot_description": open(urdf, "r").read()},
                    ],
                )
            ],
        ),

        # Robot spawn (WSL-ben kellhet nagyobb késleltetés)
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    output="screen",
                    arguments=[
                        "-entity", "turtlebot3_burger",
                        "-file", model_sdf,
                        "-x", "0", "-y", "0", "-z", "0.01",
                    ],
                )
            ],
        ),

        # A TE controller node-od (kicsit később, hogy a /scan már biztosan éljen)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package="autonomous_robot",
                    executable="controller_node",
                    name="controller_node",
                    output="screen",
                    parameters=[
                        {"stop_distance": 0.5},
                        {"forward_speed": 0.2},
                        {"turn_speed": 0.8},
                        {"use_sim_time": True},
                    ],
                )
            ],
        ),
    ])

