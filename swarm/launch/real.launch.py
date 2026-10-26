from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_real_setup(context, *args, **kwargs):
    drone_id = int(LaunchConfiguration('drone_id').perform(context))
    fcu_url = LaunchConfiguration('fcu_url').perform(context).strip()
    tgt_system = LaunchConfiguration('tgt_system').perform(context).strip()
    lidar_z = LaunchConfiguration('lidar_z').perform(context).strip()

    drone_name = f'drone{drone_id}'
    sysid = str(drone_id + 1)

    # 1. Anchor 'world' -> 'map' with wall clock (use_sim_time=False)
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{drone_name}_world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': False}],
    )

    # 2. Static TF: droneX/base_link -> droneX/lidar_link
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{drone_name}_base_to_lidar_tf',
        arguments=['0', '0', lidar_z, '0', '0', '0', f'{drone_name}/base_link', f'{drone_name}/lidar_link'],
        parameters=[{'use_sim_time': False}],
    )

    # 3. Hardware MAVROS Node (Cube Orange+ via USB or TELEM2 UART)
    mavros_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'mavros', 'apm.launch',
            f'fcu_url:={fcu_url}',
            f'namespace:={drone_name}/mavros',
            f'uas_prefix:=/uas{sysid}',
            f'tgt_system:={tgt_system}',
        ],
        output='screen',
    )

    return [world_to_map_tf, base_to_lidar_tf, mavros_cmd]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='0', description='Drone index (0 -> /drone0)'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600', description='Serial/UDP FCU connection'),
        DeclareLaunchArgument('tgt_system', default_value='1', description='MAVLink SYSID_THISMAV of the FCU'),
        DeclareLaunchArgument('lidar_z', default_value='0.08', description='LiDAR height offset above base_link (m)'),
        OpaqueFunction(function=launch_real_setup),
    ])