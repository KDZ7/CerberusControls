import os
from relaymd.relay import Helpers
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart
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

            rviz:
                config: [rviz config file] (default: rviz.yaml)

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
    cfgcmanager = PathJoinSubstitution([FindPackageShare("roscon"), "setup", "cmanager.yaml"])
    display_process = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), "launch", "display", "display.launch.py", f"relay:={os.path.join(get_package_share_directory('roscon'), 'config', 'relay.yaml')}"],
        output="screen"
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=in_dict["namespace"],
        name="controller_manager",
        parameters=[cfgcmanager],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen"
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=in_dict["namespace"],
        name="joint_state_broadcaster_spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )
    IMUC0_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=in_dict["namespace"],
        name="IMUC0_sensor_broadcaster_spawner",
        arguments=["IMUC0_sensor_broadcaster", "--load-only"],
        output="screen"
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=in_dict["namespace"],
        name="joint_trajectory_controller_spawner",
        arguments=["joint_trajectory_controller"],
        output="screen"
    )
    joint_group_positon_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=in_dict["namespace"],
        name="joint_group_positon_controller_spawner",
        arguments=["joint_group_position_controller", "--load-only"],
        output="screen"
    )
    rqt_joint_trajectory_controller = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        namespace=in_dict["namespace"],
        name='rqt_joint_trajectory_controller',
        output='screen'
    )
    rqt_controller_manager = Node(
        package="rqt_controller_manager",   
        executable="rqt_controller_manager",
        namespace=in_dict["namespace"],
        name="rqt_controller_manager",
        output="screen"
    )
    ik_solver = Node(
        package="ik_solver",
        executable="ik_solver",
        namespace=in_dict["namespace"],
        name="ik_solver",
        output="screen"
    )
    zmp_stabilizer = Node(
        package="zmp_stabilizer",
        executable="zmp_stabilizer",
        namespace=in_dict["namespace"],
        name="zmp_stabilizer",
        output="screen",
        remappings=[("zmp_stabilizer/validate_positions", "joint_trajectory_controller/commands")]
    )
    event_handler = [
        RegisterEventHandler(OnProcessStart(target_action=display_process, on_start=[TimerAction(period=10.0, actions=[joint_state_broadcaster_spawner])])),
        RegisterEventHandler(OnProcessStart(target_action=joint_state_broadcaster_spawner, on_start=[TimerAction(period=5.0, actions=[joint_trajectory_controller_spawner])])),
        RegisterEventHandler(OnProcessStart(target_action=joint_trajectory_controller_spawner, on_start=[TimerAction(period=5.0, actions=[joint_group_positon_controller_spawner])])),
        RegisterEventHandler(OnProcessStart(target_action=joint_group_positon_controller_spawner, on_start=[TimerAction(period=5.0, actions=[rqt_joint_trajectory_controller])])),
        RegisterEventHandler(OnProcessStart(target_action=rqt_joint_trajectory_controller, on_start=[TimerAction(period=5.0, actions=[rqt_controller_manager])]))
    ]
    return LaunchDescription([_cfgrelay, display_process, *event_handler, ik_solver, zmp_stabilizer])