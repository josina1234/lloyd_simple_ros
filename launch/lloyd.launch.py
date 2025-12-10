from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushROSNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

# needed changes:
# for changed namespace, create new rviz config file with bluerov01 tag
# launch hardware on klopsi-main, klopsi-buddy
# launch visual localization
# launch hippo_common tf_publisher_bluerov

# topics zum positioncontroller publishen in termianl: ros2 topic pub -r 50 /bluerov01/position_controller/setpoint geometry_msgs/msg/PointStamped "{point:{x: 1.4, y: 1.5}}"


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    # arg = DeclareLaunchArgument('vehicle_name')
    # launch_description.add_action(arg)

    # Argument für die Liste der Fahrzeugnamen
    vehicle_names_arg = DeclareLaunchArgument(
        'vehicle_names',
        default_value='bluerov00,bluerov01',
        description='Comma-separated list of vehicle names/namespaces to spawn.'
    )
    launch_description.add_action(vehicle_names_arg)

    # use_sim_time launch argument
    arg = DeclareLaunchArgument('use_sim_time')
    launch_description.add_action(arg)

    # dynamische Gruppenerstellung basierend auf vehicle_names
    launch_description.add_action(OpaqueFunction(function=add_auv_groups))

    # RViz2 Node
    rviz_file = str(
        get_package_share_path('lloyd_simple') / 'config/rviz.rviz')

    action = Node(
        executable='rviz2',
        package='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file, '--ros-args', '--log-level', 'error'],
    )
    launch_description.add_action(action)

    return launch_description


def add_auv_groups(context, *args, **kwargs):
    """Erstellt dynamisch GroupActions für jedes Fahrzeug"""
    # Package-Pfade
    package_path = get_package_share_path('lloyd_simple')
    position_controller_params_file_path = str(
        package_path / 'config/position_controller_config.yaml')
    yaw_controller_params_file_path = str(package_path /
                                          'config/yaw_controller_config.yaml')
    path_follower_params_file_path = str(package_path /
                                         'config/path_follower_config.yaml')
    barriers_file_path = str(package_path / 'config/barriers.yaml')
    lloyd_parameters_file_path = str(package_path / 'config/lloyd_params.yaml')

    # Zugriff auf die vehicle_names aus dem LaunchConfiguration
    vehicle_names_str = context.launch_configurations['vehicle_names']
    vehicle_names = [name.strip() for name in vehicle_names_str.split(',')]

    num_vehicles = len(vehicle_names)

    # Add start_algorithm node here where we have context
    start_algorithm_node = Node(
        executable='start_algorithm.py',
        package='lloyd_simple',
        name='start_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'num_vehicles': num_vehicles,
            'vehicle_names': vehicle_names,
            },
            lloyd_parameters_file_path,
            barriers_file_path
        ],
    )

    groups = []

    # Für jedes Fahrzeug eine Gruppe erstellen
    for vehicle_name in vehicle_names:
        group = GroupAction([
            PushROSNamespace(vehicle_name),

            # # lloyd algorithm for path planning
            Node(
                executable='lloyd_path_planner.py',
                package='lloyd_simple',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'num_vehicles': num_vehicles,
                        'vehicle_name': vehicle_name,
                    },
                    barriers_file_path,
                    lloyd_parameters_file_path,
                ],
                output='screen',
                emulate_tty=True,
            ),
            Node(
                executable='robot_marker_publisher',
                package='fav',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    },
                ],
            ),
            # # computes a smooth trajectory between the waypoints, and yaw angles
            # Node(
            #     executable='path_follower.py',
            #     package='lloyd_simple',
            #     parameters=[
            #         {
            #             'use_sim_time': LaunchConfiguration('use_sim_time'),
            #         },
            #         path_follower_params_file_path,
            #     ],
            # ),
            ## position controller
            Node(
                executable='position_controller.py',
                package='lloyd_simple',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'vehicle_name': vehicle_name,
                    },
                    position_controller_params_file_path,
                ],
                output='screen',
            ),
            ## yaw controller
            Node(
                executable='yaw_controller.py',
                package='lloyd_simple',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'vehicle_name': vehicle_name,
                    },
                    yaw_controller_params_file_path,
                ],
                output='screen',
            ),
        ])
        groups.append(group)

    return [start_algorithm_node] + groups
