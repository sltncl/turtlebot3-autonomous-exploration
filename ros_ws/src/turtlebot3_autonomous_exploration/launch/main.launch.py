from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Paths to external launch files and configuration files ===
    gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch', 'turtlebot3_custom_house.launch.py'  # Launch file for Gazebo simulation
    )
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py'  # Launch file for SLAM Toolbox (online async mode)
    )
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py'  # Launch file for Nav2 navigation stack
    )
    nav2_config_file = os.path.join(
        get_package_share_directory('turtlebot3_autonomous_exploration'),
        'config', 'nav2.yaml'  # Nav2 parameters customized for TurtleBot3 Burger
    )
    slam_config_file = os.path.join(
        get_package_share_directory('turtlebot3_autonomous_exploration'),
        'config', 'slam.yaml'  # Custom SLAM Toolbox parameters
    )
    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_autonomous_exploration'),
        'rviz', 'tb3_main.rviz'  # RViz configuration file
    )

    # === Launch Gazebo simulation ===
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'on_exit_shutdown': 'True'}.items()  # Shutdown everything if Gazebo is closed
    )

    # === Launch SLAM Toolbox with simulation time and custom parameters ===
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'use_sim_time': 'true',  # Use Gazebo simulated time
            'params_file': slam_config_file  # Load SLAM config
        }.items()
    )

    # === Launch Navigation2 stack with parameters and simulation time ===
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_config_file  # Load Nav2 config
        }.items()
    )

    # === Launch RViz for visualization ===
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],  # Open with custom RViz config
        output='screen',
        parameters=[{'use_sim_time': True}]  # Synchronize with simulation time
    )

    # === Launch frontier detector node (detects unexplored frontiers) ===
    frontier_detector_node = Node(
        package='turtlebot3_autonomous_exploration',
        executable='frontier_detector_node',
        name='frontier_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # === Launch frontier goal publisher node (sends goals to frontiers) ===
    frontier_goal_publisher_node = Node(
        package='turtlebot3_autonomous_exploration',
        executable='frontier_goal_publisher_node',
        name='frontier_goal_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ) 

    # === Launch EKF node for sensor fusion ===
    ekf_config_file = os.path.join(
        get_package_share_directory('turtlebot3_autonomous_exploration'),
        'config', 'ekf.yaml'  # Configuration file for robot_localization
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}],
        remappings=[
            ('/odometry/filtered', '/odom/filtered')  # Publish EKF output under /odom/filtered
        ]
    )

    # === Return full launch description ===
    return LaunchDescription([
        gazebo,
        rviz,
        slam,
        ekf_node,
        nav2,
        frontier_detector_node,
        frontier_goal_publisher_node,
    ])