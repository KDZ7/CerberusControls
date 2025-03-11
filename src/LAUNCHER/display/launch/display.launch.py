import os
import re
from relaymd.relay import Relay
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def addEnvironment(key, tuple=None):
    if tuple is not None and Relay.dict is not None:
        value = Relay.dict
        for item in tuple:
            value = value.get(item, None)
        current = os.environ.get(key, "")
        if current:
            os.environ[key] = current + os.pathsep + str(value)
        else:
            os.environ[key] = str(value)

def bindContext(context):
    if "pkg_robot_description" in Relay.dict and Relay.dict["pkg_robot_description"] is not None:
        Relay.dict["robot_description"] = Command([" xacro ", PathJoinSubstitution([FindPackageShare(Relay.dict["pkg_robot_description"]), Relay.dict["robot_description"]])])
    else:
        Relay.dict["robot_description"] = Command([" xacro ", Relay.dict["robot_description"]])
    
    regex_fetch_package = re.compile(r'\[([^\]]+)\]')

    if "controller_manager" in Relay.dict:
        setup_path = Relay.dict["controller_manager"]["setup_path"]
        if setup_path is not None:
            matches =  regex_fetch_package.findall(setup_path)
            for match in matches:
                setup_path = setup_path.replace("[" + match + "]", FindPackageShare(match).perform(context))
            Relay.dict["controller_manager"]["setup_path"] = setup_path
    
    if "rviz" in Relay.dict:
        config_path = Relay.dict["rviz"]["config_path"]
        matches =  regex_fetch_package.findall(config_path)
        for match in matches:
            config_path = config_path.replace("[" + match + "]", FindPackageShare(match).perform(context))
        Relay.dict["rviz"]["config_path"] = config_path

    if "gazebo" in Relay.dict and Relay.dict["gazebo"]["gzbridge_path"] is not None:
        gzbridge_path = Relay.dict["gazebo"]["gzbridge_path"]
        matches =  regex_fetch_package.findall(gzbridge_path)
        for match in matches:
            gzbridge_path = gzbridge_path.replace("[" + match + "]", FindPackageShare(match).perform(context))
        Relay.dict["gazebo"]["gzbridge_path"] = gzbridge_path

    if "environment" in Relay.dict:
        res_paths = Relay.dict["environment"]["res_paths"]
        matches =  regex_fetch_package.findall(res_paths)
        for match in matches:
            res_paths = res_paths.replace("[" + match + "]", FindPackageShare(match).perform(context))
        plug_paths = Relay.dict["environment"]["plug_paths"]
        matches =  regex_fetch_package.findall(plug_paths)
        for match in matches:
            plug_paths = plug_paths.replace("[" + match + "]", FindPackageShare(match).perform(context))
        lib_paths = Relay.dict["environment"]["lib_paths"]
        matches =  regex_fetch_package.findall(lib_paths)
        for match in matches:
            lib_paths = lib_paths.replace("[" + match + "]", FindPackageShare(match).perform(context))
        Relay.dict["environment"]["res_paths"] = res_paths
        Relay.dict["environment"]["plug_paths"] = plug_paths
        Relay.dict["environment"]["lib_paths"] = lib_paths

def exec(context):
    tasks = []
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=Relay.dict["namespace"],
        name="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(Relay.dict["robot_description"], value_type=str)},
            {"use_sim_time": Relay.dict["use_sim_time"]}
        ],
        output="screen",
        remappings=[("joint_states", "/csystem/joint_states")]
        # remappings=[("joint_states", "/joint_states")]
    )
    tasks.append(robot_state_publisher)

    if Relay.dict["controller_manager"]["use"]:
        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=Relay.dict["namespace"],
            name="controller_manager",
            parameters=[
                {
                    "robot_description": ParameterValue(Relay.dict["robot_description"], value_type=str),
                    "use_sim_time": Relay.dict["use_sim_time"],
                    "update_rate": Relay.dict["controller_manager"]["update_rate"]
                },
                Relay.dict["controller_manager"]["setup_path"]
            ],
            output="screen"
        )
        tasks.append(controller_manager)

    if Relay.dict["joint_state_publisher"] and Relay.dict["display_on"] != "gazebo":
        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            namespace=Relay.dict["namespace"],
            name="joint_state_publisher_gui",
            output="screen"
        )
        tasks.append(joint_state_publisher_gui)

    if "rviz" in Relay.dict and Relay.dict["display_on"] == "rviz" or Relay.dict["display_on"] == "both":
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            namespace=Relay.dict["namespace"],
            name="rviz2",
            arguments=["-d", Relay.dict["rviz"]["config_path"]],
            output="screen"
        )
        tasks.append(rviz)

    if "gazebo" in Relay.dict and Relay.dict["display_on"] == "gazebo" or Relay.dict["display_on"] == "both":
        if Relay.dict["gazebo"]["mode"] == "server" or Relay.dict["gazebo"]["mode"] == "both":
            gazebo_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
                launch_arguments={
                    "gz_args": [
                        " -s -v ", 
                        " -r " if Relay.dict["gazebo"]["autorun"] else "",
                        PathJoinSubstitution([Relay.dict["gazebo"]["world"]])
                    ],
                    "on_exit_shutdown": "true",
                }.items()
            )
            tasks.append(gazebo_server)

        if Relay.dict["gazebo"]["mode"] == "client" or Relay.dict["gazebo"]["mode"] == "both":
            spawn_robot = Node(
                package="ros_gz_sim",
                executable="create",
                namespace=Relay.dict["namespace"],
                name=Relay.dict["gazebo"]["robot_name"],
                arguments = [
                    "-name", Relay.dict["gazebo"]["robot_name"],
                    "-string", Relay.dict["robot_description"],
                    "-x", str(Relay.dict["gazebo"]["robot_position"][0]),
                    "-y", str(Relay.dict["gazebo"]["robot_position"][1]),
                    "-z", str(Relay.dict["gazebo"]["robot_position"][2]),
                    "-R", str(Relay.dict["gazebo"]["robot_position"][3]),
                    "-P", str(Relay.dict["gazebo"]["robot_position"][4]),
                    "-Y", str(Relay.dict["gazebo"]["robot_position"][5]),
                ],
                output="screen"
            )
            tasks.append(spawn_robot)

            gazebo_client = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
                launch_arguments={
                    "gz_args": "-g",
                    "on_exit_shutdown": "true",
                }.items()
            )
            tasks.append(gazebo_client)

            if "gzbridge_path" in Relay.dict["gazebo"] and Relay.dict["gazebo"]["gzbridge_path"] is not None:
                gzbridge = Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    namespace=Relay.dict["namespace"],
                    name="gzbridge",
                    arguments=[
                        "--ros-args", 
                        "-p", 
                        f"config_file:={Relay.dict['gazebo']['gzbridge_path']}"
                    ],
                    output="screen"
                )
                tasks.append(gzbridge)

    return tasks

def generate_launch_description():
    cfgrelay = LaunchConfiguration("relay")
    _cfgrelay = DeclareLaunchArgument(
        "relay",
        default_value=PathJoinSubstitution([FindPackageShare("display"), "config", "relay.yaml"]),
        description="""
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
    
    actions = []
    actions.append(_cfgrelay)
    actions.append(Relay.read(cfgrelay))
    actions.append(Relay.action(bindContext))
    actions.append(Relay.action(lambda _: addEnvironment("ROS_DOMAIN_ID", ("ros_domain_id",))))
    actions.append(Relay.action(lambda _: addEnvironment("GZ_SIM_RESOURCE_PATH", ("environment", "res_paths"))))
    actions.append(Relay.action(lambda _: addEnvironment("GZ_SIM_SYSTEM_PLUGIN_PATH", ("environment", "plug_paths"))))
    actions.append(Relay.action(lambda _: addEnvironment("LD_LIBRARY_PATH", ("environment", "lib_paths"))))
    actions.append(ExecuteProcess(cmd=["bash", "-c", "printenv | grep -E 'ROS_DOMAIN_ID|GZ_SIM_RESOURCE_PATH|GZ_SIM_SYSTEM_PLUGIN_PATH|LD_LIBRARY_PATH|MESA_D3D12_DEFAULT_ADAPTER_NAME'"], output="screen"))
    actions.append(Relay.action(exec))
    return LaunchDescription([_cfgrelay, *actions])
