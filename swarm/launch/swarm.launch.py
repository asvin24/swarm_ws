import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    home_dir = os.path.expanduser('~')
    repo_root = os.path.join(home_dir, 'ardupilot_ws', 'src', 'swarm_ws', 'swarm')
    config_path = os.path.join(repo_root, 'config', 'swarm.yaml')
    xacro_path = os.path.join(repo_root, 'models', 'iris_with_gimbal_custom', 'model.sdf.xacro')
    worlds_dir = os.path.join(repo_root, 'worlds')

    # 1. Resolve 'world' launch argument ("normal" vs "maze")
    world_arg = LaunchConfiguration('world').perform(context).strip().lower()
    if world_arg in ('maze', 'world_with_maze', 'world_with_maze.sdf'):
        world_file = 'world_with_maze.sdf'
    elif world_arg in ('normal', 'artemis', 'artemis_world', 'artemis_world.sdf'):
        world_file = 'artemis_world.sdf'
    else:
        world_file = world_arg if world_arg.endswith('.sdf') else f'{world_arg}.sdf'

    world_path = os.path.join(worlds_dir, world_file)

    # 2. Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-v 4 -r {world_path}'}.items()
    )
    entities = [gz_sim]

    # 3. Read spacing from YAML if available
    yaml_spacing = 2.0
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file) or {}
            yaml_spacing = config.get('swarm', {}).get('y_spacing', 2.0)

    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    y_spacing_str = LaunchConfiguration('y_spacing').perform(context)
    spacing = float(y_spacing_str) if y_spacing_str else float(yaml_spacing)

    # Check if ArduCopter SITL binary is already compiled so we can skip waf rebuilds (-N)
    arducopter_bin = os.path.join(home_dir, 'ardupilot_ws', 'src', 'ardupilot', 'build', 'sitl', 'bin', 'arducopter')
    has_sitl_bin = os.path.exists(arducopter_bin)

    # 4. Spawn each drone + SITL + MAVROS + LiDAR Bridge
    for i in range(num_drones):
        drone_name = f'drone{i}'
        sysid = str(i + 1)
        fdm_in = str(9002 + (i * 10))
        mavlink_port = str(14550 + (i * 10))
        y_pos = str(i * spacing)

        # A) Gazebo Spawn Node
        drone_xml = Command([
            'xacro ', xacro_path,
            ' name:=', drone_name,
            ' fdm_port_in:=', fdm_in
        ])
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', drone_xml,
                '-name', drone_name,
                '-x', '0.0',
                '-y', y_pos,
                '-z', '0.2'
            ],
            output='screen'
        )
        entities.append(spawn_node)

        # B) ArduPilot SITL Process
        sitl_args = [
            'sim_vehicle.py', '-v', 'ArduCopter', '-f', 'gazebo-iris',
            '--model', 'JSON', '-I', str(i),
            '--no-mavproxy',
            '-A', f'--serial0=udpclient:127.0.0.1:{mavlink_port}',
        ]
        if has_sitl_bin:
            sitl_args.append('-N')

        sitl_cmd = ExecuteProcess(
            cmd=sitl_args,
            output='screen'
        )
        entities.append(sitl_cmd)

        # C) MAVROS Node
        fcu_url = f'udp://127.0.0.1:{mavlink_port}@'
        mavros_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'mavros', 'apm.launch',
                f'fcu_url:={fcu_url}',
                f'namespace:={drone_name}/mavros',
                f'uas_prefix:=/uas{sysid}',
                'tgt_system:=1'
            ],
            output='screen'
        )
        entities.append(mavros_cmd)

        # D) 2D LiDAR ROS-Gazebo Bridge (/droneX/scan)
        lidar_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{drone_name}_lidar_bridge',
            arguments=[
                f'/{drone_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
            output='screen'
        )
        entities.append(lidar_bridge)

    return entities


def generate_launch_description():
    home_dir = os.path.expanduser('~')
    repo_root = os.path.join(home_dir, 'ardupilot_ws', 'src', 'swarm_ws', 'swarm')
    models_dir = os.path.join(repo_root, 'models')
    worlds_dir = os.path.join(repo_root, 'worlds')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='normal',
        description='World to load: "normal" (artemis_world.sdf) or "maze" (world_with_maze.sdf)'
    )

    declare_num_drones = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones to spawn in a straight line along the Y-axis'
    )

    declare_y_spacing = DeclareLaunchArgument(
        'y_spacing',
        default_value='',
        description='Spacing between drones along the Y-axis'
    )

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = f"{models_dir}:{worlds_dir}:{existing_gz_path}" if existing_gz_path else f"{models_dir}:{worlds_dir}"

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_path
    )

    return LaunchDescription([
        declare_world,
        declare_num_drones,
        declare_y_spacing,
        set_env,
        OpaqueFunction(function=launch_setup)
    ])