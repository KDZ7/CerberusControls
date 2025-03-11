import os
from relaymd.relay import Helpers
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    _cfgrelay = DeclareLaunchArgument(
        "Edit Relay",
        default_value="*",
        description="""
            =================================================================
             CERBERUS CONFIGURATION FILE EDITOR - [roscon]/config/relay.yaml
            =================================================================
            
            ros_domain_id: [ROS domain id] (default: 0)
            namespace: [namespace] (default: Null)
            pkg_robot_description: [package name of robot description file] (default: Null)
            robot_description: [path to robot description file]
            display_on: [rviz/gazebo/both]
            use_sim_time: [use simulation time] (default: false)

            controller_manager:
                use: [use controller manager] (default: false)
                setup_path: [path to controller manager setup file]
                use_sim_time: [controller manager use simulation time] (default: false)
                update_rate: [update rate of controller manager] (default: 100)

            rviz:
                config_path: [path to rviz configuration file]

            gazebo:
                mode: [client/server/both]
                world: [model file of world] (default: empty.world.sdf)
                robot_name: [name of robot]
                robot_position: [initial position of robot: [x, y, z, roll, pitch, yaw]]
                gzbridge_path: [path to gazebo bridge configuration file] (default: Null)
                autorun: [run the simulation automatically] (default: false)

            environment:
                res_paths: [paths to the resources] ex: [ros_package1]/models/sdf:[ros_package2]/models/urdf:/absolute/path/models/urdf etc ..
                plug_paths: [paths to the plugins] ex: [ros_package1]/lib:[ros_package2]/lib:/absolute/path/lib etc ..
                lib_paths: [paths to the libraries] ex: [ros_package1]/lib:[ros_package2]/lib:/absolute/path/lib etc ..
            """
    )
    in_dict = Helpers.load_yaml(os.path.join(get_package_share_directory("roscon"), "config", "relay.yaml"))
    display_process = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"), "launch", "display", "display.launch.py", f"relay:={os.path.join(get_package_share_directory("roscon"), "config", "relay.yaml")}"],
        output="screen"
    )
    csystem_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=in_dict["namespace"],
        name="csystem_controller_spawner",
        arguments=["csystem_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace=in_dict["namespace"],
        name="joint_state_publisher_gui",
        output="screen",
        remappings=[("joint_states", "/csystem/joint_commands")]
    )
    iksolver = Node(
        package="iksolver",
        executable="iksolver",
        namespace=in_dict["namespace"],
        name="iksolver",
        output="screen",
        remappings=[
            ("~/waypoint", "/waypoint"),
            ("~/joint_states", "/csystem/joint_states"),
        ]
    )
    test_iksolver = Node(
        package="test_iksolver",
        executable="target",
        namespace=in_dict["namespace"],
        name="target",
        parameters=[{"base_frame": "M"}],
        output="screen",
        remappings=[
            ("~/waypoint", "/waypoint"),
            ("~/target", "/clicked_point")
        ]
    )
    rqt_controller_manager = Node(
        package="rqt_controller_manager",   
        executable="rqt_controller_manager",
        namespace=in_dict["namespace"],
        name="rqt_controller_manager",
        output="screen"
    )
    # zmp_stabilizer = Node(
    #     package="zmp_stabilizer",
    #     executable="zmp_stabilizer",
    #     namespace=in_dict["namespace"],
    #     name="zmp_stabilizer",
    #     output="screen",
    #     remappings=[("zmp_stabilizer/validate_positions", "joint_trajectory_controller/commands")]
    # )
    # event_handler = [
    #     RegisterEventHandler(OnProcessStart(target_action=display_process, on_start=[TimerAction(period=1.0, actions=[csystem_controller_spawner])])),
    #     RegisterEventHandler(OnProcessStart(target_action=csystem_controller_spawner, on_start=[TimerAction(period=1.0, actions=[joint_state_publisher_gui])])),
    #     RegisterEventHandler(OnProcessStart(target_action=joint_state_publisher_gui, on_start=[TimerAction(period=1.0, actions=[iksolver])])),
    #     RegisterEventHandler(OnProcessStart(target_action=iksolver, on_start=[TimerAction(period=1.0, actions=[test_iksolver])])),
    #     RegisterEventHandler(OnProcessStart(target_action=test_iksolver, on_start=[TimerAction(period=1.0, actions=[rqt_controller_manager])]))
    # ]

    event_handler = [
        RegisterEventHandler(OnProcessStart(target_action=display_process, on_start=[csystem_controller_spawner])),
        RegisterEventHandler(OnProcessStart(target_action=csystem_controller_spawner, on_start=[joint_state_publisher_gui])),
        RegisterEventHandler(OnProcessStart(target_action=joint_state_publisher_gui, on_start=[iksolver])),
        RegisterEventHandler(OnProcessStart(target_action=iksolver, on_start=[test_iksolver])),
    ]
    return LaunchDescription([_cfgrelay, display_process, *event_handler])