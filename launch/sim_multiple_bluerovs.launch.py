from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

#########
# when launching this file with vehicle_names
# ros2 launch lloyd_simple sim_multiple_bluerovs.launch.py vehicle_names:="bluerov00,bluerov01"
# bis zu 4 Fahrzeuge möglich
# default: "bluerov00,bluerov01"
# start_gui:=false  # um Gazebo ohne GUI zu starten
# ros2 launch lloyd_simple sim_multiple_bluerovs.launch.py vehicle_names:="bluerov00,bluerov01" start_gui:=false
#########

#########
# change spawn positions in the function spawn_bluerovs
#########

def add_keyboard_control_node(self, context, launch_description: LaunchDescription) -> None:
    names = context.launch_configurations['vehicle_names'].split(',')
    self.get_logger().info(f"Vehicle names for keyboard control: {names}")
    first_namespace = names[0].strip() if names else 'bluerov00'
    second_namespace = names[1].strip() if names else 'bluerov01'
    third_namespace = names[2].strip() if len(names) > 2 else 'bluerov02'

    # action = Node(
    #     executable='keyboard_control_one.py',
    #     package='bachelor_arbeit_test',
    #     name='keyboard_control_one',
    #     namespace=first_namespace,  # Use the first vehicle in the list for keyboard control
    #     on_exit=Shutdown(),
    # )
    # print(f"Starting keyboard control one in namespace: {first_namespace}")
    # launch_description.add_action(action)

    # action = Node(
    #     executable='keyboard_control_two.py',
    #     package='bachelor_arbeit_test',
    #     name='keyboard_control_two',
    #     namespace=second_namespace,  # Use the second vehicle in the list for keyboard control
    #     on_exit=Shutdown(),
    # )
    # print(f"Starting keyboard control two in namespace: {second_namespace}")
    # launch_description.add_action(action)

def generate_launch_description():
    # Argument für die Liste der Fahrzeugnamen
    vehicle_names_arg = DeclareLaunchArgument(
        'vehicle_names',
        default_value='bluerov00,bluerov01',
        description='Comma-separated string of vehicle names/namespaces to spawn.'
    )
    start_gui_arg = DeclareLaunchArgument(
        'start_gui',
        default_value='true',
        description='Start the gazebo GUI. Otherwise gazebo will run in headless mode.'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(vehicle_names_arg)
    launch_description.add_action(start_gui_arg)

    # Start Gazebo
    hippo_sim_path = get_package_share_path('hippo_sim')
    gazebo_launch_path = str(hippo_sim_path / 'launch/start_gazebo.launch.py')
    gazebo_args = {
        'start_gui': LaunchConfiguration('start_gui')
    }
    gazebo_source = PythonLaunchDescriptionSource(gazebo_launch_path)
    launch_description.add_action(
        IncludeLaunchDescription(gazebo_source, launch_arguments=gazebo_args.items())
    )

    # Spawn n BlueROVs
    lloyd_simple_path = get_package_share_path('lloyd_simple')
    def spawn_bluerovs(context):
        
        names = context.launch_configurations['vehicle_names'] # here still of type string
        if len(context.launch_configurations['vehicle_names']) > 10:
            names = names.split(',') # convert to list if multiple names are given
            # exception wenn mehr als 4 fahrzeuge
            if len(names) > 4:
                raise RuntimeError(f"You tried to spawn {len(names)} vehicles, but the maximum is 4.")
            # exception wenn namen nicht eindeutig
            if len(set(names)) != len(names):
                raise RuntimeError(f"Vehicle names must be unique. You provided: {names}")
        
        # TODO hier Paramter zum Spawnen anpassen
        xi = [0.8, 1.0, 0.5, 1.5] # Spawnpositionen x für bis zu 4 Fahrzeuge
        yi = [0.8, 0.8, 1.5, 1.5] 
        zi = [-0.5, -0.5, -0.5, -0.5]
        Yi = [1.5708, 1.5708, 1.5708, 1.5708] # Spawnwinkel (Yaw) für bis zu 4 Fahrzeuge
        actions = []
        spawn_path = str(lloyd_simple_path / 'launch/spawn_bluerov.launch.py') # pfad zur abgeänderten Launchfile (original: hippo_sim)

        if type(names) == list:
            for i, name in enumerate(names):
                args = {
                    'vehicle_name': TextSubstitution(text=name.strip()),
                    'use_sim_time': TextSubstitution(text='true'),
                    'x' : TextSubstitution(text=str(xi[i])),
                    'y' : TextSubstitution(text=str(yi[i])),
                    'z' : TextSubstitution(text=str(zi[i])),
                    'Y' : TextSubstitution(text=str(Yi[i])),
                }
                source = PythonLaunchDescriptionSource(spawn_path)
                actions.append(
                IncludeLaunchDescription(source, launch_arguments=args.items())
                )   
        else:
            args = {
                'vehicle_name': TextSubstitution(text=names.strip()),
                'use_sim_time': TextSubstitution(text='true'),
                'x' : TextSubstitution(text=str(xi[0])),
                'y' : TextSubstitution(text=str(yi[0])),
                'z' : TextSubstitution(text=str(zi[0])),
                'Y' : TextSubstitution(text=str(Yi[0])),
            }
            source = PythonLaunchDescriptionSource(spawn_path)
            actions.append(
                IncludeLaunchDescription(source, launch_arguments=args.items())
            )
    
        return actions

    launch_description.add_action(OpaqueFunction(function=spawn_bluerovs))

    # genaue funktion von opaquefunction? --> kann dynamisch auf den Launchcontext zugreifen und auswerten

    return launch_description