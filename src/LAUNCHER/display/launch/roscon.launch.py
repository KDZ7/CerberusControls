import os
from relaymd.relay import Helpers
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import  FindExecutable, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
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
    urdf_xacro_content = Command([" xacro ", os.path.join(get_package_share_directory(in_dict["pkg_robot_description"]), in_dict["robot_description"])])
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
    # # With Gazebo
    # csystem_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=in_dict["namespace"],
    #     name="csystem_controller_spawner",
    #     arguments=["csystem_controller", "--controller-manager", "/controller_manager", "--load-only"],
    #     output="screen"
    # )
    # transistion_csystem_controller_spawner = ExecuteProcess(
    #     cmd=["sleep 5 && ros2 control set_controller_state csystem_controller inactive && sleep 2 && ros2 control set_controller_state csystem_controller active"],
    #     output="screen",
    #     shell=True
    # )
    
    # joint_state_publisher_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     namespace=in_dict["namespace"],
    #     name="joint_state_publisher_gui",
    #     output="screen",
    #     remappings=[("joint_states", "/csystem/joint_commands")]
    # )

    iksolver = Node(
        package="iksolver",
        executable="iksolver",
        namespace=in_dict["namespace"],
        name="iksolver",
        parameters=[{
            "config_file_path": os.path.join(get_package_share_directory("iksolver"), "config", "ikconfig.yaml"),
            "urdf_content": ParameterValue(urdf_xacro_content, value_type=str)
        }],
        output="screen",
        remappings=[
            ("~/waypoint", "/waypoint"),
            ("~/joint_states", "/csystem/joint_states"),
            ("~/solutions", "/solutions/in")
        ]
    )
    transistion_iksolver = ExecuteProcess(
        cmd=["sleep 2 && ros2 lifecycle set /iksolver configure && sleep 2 && ros2 lifecycle set /iksolver activate"],
        shell=True
    )
    iksolver_bridge = Node(
        package="iksolver_bridge",
        executable="iksolver_bridge",
        namespace=in_dict["namespace"],
        name="iksolver_bridge",
        parameters=[{"config_file_path": os.path.join(get_package_share_directory("iksolver_bridge"), "config", "iksolver_bridge_config.yaml")}],
        output="screen",
        remappings=[
            ("~/in",  "/solutions/in"),
            ("~/out", "/solutions/out")
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
    gmotion = Node(
        package="gmotion",
        executable="gmotion",
        namespace=in_dict["namespace"],
        name="gmotion",
        parameters=[{"config_file_path": os.path.join(get_package_share_directory("gmotion"), "config", "gmotion.yaml")}],
        output="screen",
        remappings=[
            ("~/command", "/command"),
            ("~/solutions", "/solutions/out"),
            ("~/in", "/csystem/joint_states"),
            ("~/out", "/csystem/joint_commands"),
            ("~/waypoint", "/waypoint")
        ]
    )
    transistion_gmotion = ExecuteProcess(
        cmd=["sleep 2 && ros2 lifecycle set /gmotion configure && sleep 2 && ros2 lifecycle set /gmotion activate"],
        shell=True
    )
    # zmp_stabilizer = Node(
    #     package="zmp_stabilizer",
    #     executable="zmp_stabilizer",
    #     namespace=in_dict["namespace"],
    #     name="zmp_stabilizer",
    #     output="screen",
    #     remappings=[("zmp_stabilizer/validate_positions", "joint_trajectory_controller/commands")]
    # )
    event_handler = [
        RegisterEventHandler(OnProcessStart(target_action=display_process, on_start=[csystem_controller_spawner])),
        # RegisterEventHandler(OnProcessStart(target_action=csystem_controller_spawner, on_start=[joint_state_publisher_gui])),
        # RegisterEventHandler(OnProcessStart(target_action=joint_state_publisher_gui, on_start=[iksolver])),
        # RegisterEventHandler(OnProcessStart(target_action=csystem_controller_spawner, on_start=[transistion_csystem_controller_spawner])),
        RegisterEventHandler(OnProcessStart(target_action=csystem_controller_spawner, on_start=[iksolver])),
        RegisterEventHandler(OnProcessStart(target_action=iksolver, on_start=[transistion_iksolver])),
        RegisterEventHandler(OnProcessStart(target_action=transistion_iksolver, on_start=[iksolver_bridge])),
        RegisterEventHandler(OnProcessStart(target_action=iksolver_bridge, on_start=[test_iksolver])),
        RegisterEventHandler(OnProcessStart(target_action=test_iksolver, on_start=[gmotion])),
        RegisterEventHandler(OnProcessStart(target_action=gmotion, on_start=[transistion_gmotion])),
    ]
    return LaunchDescription([_cfgrelay, display_process, *event_handler])