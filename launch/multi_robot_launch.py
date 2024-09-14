import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    AppendEnvironmentVariable,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'
    
    parking_dir = get_package_share_directory('parking_lot_cleaner')
    bringup_dir = get_package_share_directory('nav2_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='True', 
        description='Whether to use simulation/Gazebo clock'
    )

    multi_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'cloned_multi_tb3_simulation_launch.py')),
        launch_arguments={
            'world': os.path.join(parking_dir, 'worlds', 'world.sdf'),
            'map': os.path.join(parking_dir, 'maps', 'parking_lot.yaml'),
            'autostart': 'True',
            'rviz_config': os.path.join(parking_dir, 'params', 'nav2_namespaced_view_rviz.yaml'),
        }.items()
    )

    robots_list = ParseMultiRobotPose('robots').value()

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction(
            [
                Node(
                    namespace=TextSubstitution(text=robot_name),
                    package='parking_lot_cleaner',
                    executable='initial_pose_publisher.py',
                    name='initial_pose_publisher_node',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'pose.x': init_pose['x']},
                        {'pose.y': init_pose['y']},
                        {'pose.z': init_pose['z']},
                        {'pose.roll': init_pose['roll']},
                        {'pose.pitch': init_pose['pitch']},
                        {'pose.yaw': init_pose['yaw']},
                    ]
                )
            ]
        )

        bringup_cmd_group.append(group)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g '}.items()
    )

    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity',
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(parking_dir, 'params', 'bridge.yaml')}',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['X3/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    hover_algorithm = Node(
        package='parking_lot_cleaner',
        executable='hover_algorithm.py',
        name='hover_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    garbage_spawner = Node(
        package='parking_lot_cleaner',
        executable='garbage_spawner.py',
        name='entity_spawner_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    garbage_deleter = Node(
        package='parking_lot_cleaner',
        executable='garbage_deleter.py',
        name='garbage_deleter_node',
        parameters=[{'use_sim_time': use_sim_time}]
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

    nav2_goal_sender = Node(
        package='parking_lot_cleaner',
        executable='nav2_goal_sender.py',
        name='nav2_goal_sender_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(parking_dir, 'models')
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(gzclient_cmd)

    ld.add_action(multi_robot_cmd)

    ld.add_action(gz_ros_bridge)
    ld.add_action(image_bridge)

    ld.add_action(hover_algorithm)
    ld.add_action(garbage_detector)
    ld.add_action(garbage_spawner)
    ld.add_action(garbage_deleter)
    ld.add_action(job_allocator)
    ld.add_action(nav2_goal_sender)

    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld
