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

WORLD_ALIASES = {
    'maze': 'world_with_maze.sdf',
    'world_with_maze': 'world_with_maze.sdf',
    'normal': 'artemis_world.sdf',
    'artemis': 'artemis_world.sdf',
    'artemis_world': 'artemis_world.sdf',
}


def resolve_spacing(context, config_path: str) -> float:
    cli_val = LaunchConfiguration('y_spacing').perform(context).strip()
    if cli_val:
        return float(cli_val)
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
            return float(cfg.get('swarm', {}).get('y_spacing', 2.0))
    return 2.0


def launch_setup(context, *args, **kwargs):
    home_dir = os.path.expanduser('~')
    swarm_share = get_package_share_directory('swarm')
    config_path = os.path.join(swarm_share, 'config', 'swarm.yaml')

    # Point to ardupilot_gazebo for all simulation-only assets
    gz_pkg_root = os.path.join(home_dir, 'ardupilot_ws', 'src', 'ardupilot_gazebo')
    xacro_path = os.path.join(gz_pkg_root, 'models', 'iris_with_gimbal_custom', 'model.sdf.xacro')
    worlds_dir = os.path.join(gz_pkg_root, 'worlds')

    raw_world = LaunchConfiguration('world').perform(context).strip().lower()
    world_file = WORLD_ALIASES.get(
        raw_world, raw_world if raw_world.endswith('.sdf') else f'{raw_world}.sdf'
    )
    world_path = os.path.join(worlds_dir, world_file)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-v 4 -r {world_path}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': True}],
    )

    entities = [gz_sim, clock_bridge, world_to_map_tf]

    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    spacing = resolve_spacing(context, config_path)
    sitl_bin = os.path.join(home_dir, 'ardupilot_ws', 'src', 'ardupilot', 'build', 'sitl', 'bin', 'arducopter')
    has_sitl_bin = os.path.exists(sitl_bin)

    for i in range(num_drones):
        drone_name = f'drone{i}'
        sysid = str(i + 1)
        fdm_port = str(9002 + (i * 10))
        mavlink_port = str(14550 + (i * 10))
        y_pos = str(i * spacing)

        drone_xml = Command(['xacro ', xacro_path, ' name:=', drone_name, ' fdm_port_in:=', fdm_port])
        entities.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-string', drone_xml, '-name', drone_name, '-x', '0.0', '-y', y_pos, '-z', '0.2'],
                output='screen',
            )
        )

        sitl_cmd = [
            'sim_vehicle.py', '-v', 'ArduCopter', '-f', 'gazebo-iris',
            '--model', 'JSON', '-I', str(i), '--no-mavproxy',
            '-A', f'--serial0=udpclient:127.0.0.1:{mavlink_port}',
        ]
        if has_sitl_bin:
            sitl_cmd.append('-N')
        entities.append(ExecuteProcess(cmd=sitl_cmd, output='screen'))

        entities.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'mavros', 'apm.launch',
                    f'fcu_url:=udp://127.0.0.1:{mavlink_port}@',
                    f'namespace:={drone_name}/mavros',
                    f'uas_prefix:=/uas{sysid}',
                    'tgt_system:=1',
                ],
                output='screen',
            )
        )

        entities.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{drone_name}_lidar_bridge',
                arguments=[f'/{drone_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                output='screen',
            )
        )
        entities.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{drone_name}_base_to_lidar_tf',
                arguments=['0', '0', '0.08', '0', '0', '0', f'{drone_name}/base_link', f'{drone_name}/lidar_link'],
                parameters=[{'use_sim_time': True}],
            )
        )

    return entities


def generate_launch_description():
    home_dir = os.path.expanduser('~')
    gz_pkg_root = os.path.join(home_dir, 'ardupilot_ws', 'src', 'ardupilot_gazebo')
    models_dir = os.path.join(gz_pkg_root, 'models')
    worlds_dir = os.path.join(gz_pkg_root, 'worlds')

    existing_gz = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_path = f'{models_dir}:{worlds_dir}:{existing_gz}' if existing_gz else f'{models_dir}:{worlds_dir}'

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='normal', description='World: "normal" or "maze"'),
        DeclareLaunchArgument('num_drones', default_value='1', description='Number of drones along Y-axis'),
        DeclareLaunchArgument('y_spacing', default_value='', description='Meters between drones along Y-axis'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_path),
        OpaqueFunction(function=launch_setup),
    ])