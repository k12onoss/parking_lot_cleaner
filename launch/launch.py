import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    x_pose = LaunchConfiguration('x_pose', default='-5.5')
    y_pose = LaunchConfiguration('y_pose', default='-1.0')

    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Package Directories
    pkg_parking_lot_cleaner = get_package_share_directory('parking_lot_cleaner')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(pkg_parking_lot_cleaner, 'worlds', 'world.sdf')
    bridge_params = os.path.join(pkg_parking_lot_cleaner, 'params', 'bridge.yaml')

    # Set gz resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_parking_lot_cleaner, 'models')]
    )

    # # Robot state publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}],
    # )

    # Gazebo Sim
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s ', world_file], 'on_exit_shutdown': 'false'}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g '}.items()
    )

    # # RViz
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(pkg_parking_lot_cleaner, 'params', 'rviz.rviz')],
    # )

    turtlebot_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Gz - ROS Bridge
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity',
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['X3/image'],
        output='screen',
    )

    hover_algorithm = Node(
        package='parking_lot_cleaner',
        executable='hover_algorithm.py',
        name='hover_node',
    )

    entity_spawner = Node(
        package='parking_lot_cleaner',
        executable='garbage_spawner.py',
        name='entity_spawner_node',
    )

    garbage_deleter = Node(
        package='parking_lot_cleaner',
        executable='garbage_deleter.py',
        name='garbage_deleter_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    image_saver = Node(
        package='parking_lot_cleaner',
        executable='image_saver.py',
        name='image_saver_node',
    )

    garbage_detector = Node(
        package='parking_lot_cleaner',
        executable='garbage_detector.py',
        name='garbage_detector_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    job_allocator = Node(
        package='parking_lot_cleaner',
        executable='job_allocator.py',
        name='job_allocator_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        [
            gz_resource_path,
            gzserver,
            gzclient,
            turtlebot_robot_state_publisher_cmd,
            spawn_turtlebot_cmd,
            gz_ros_bridge,
            image_bridge,
            # robot_state_publisher,
            hover_algorithm,
            entity_spawner,
            garbage_deleter,
            # image_saver,
            garbage_detector,
            job_allocator,
        ]
    )
