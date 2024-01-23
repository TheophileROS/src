import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='eurobot_launch'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','sim.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazerbo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
        )]),
        launch_arguments={'world': '/home/rosubuntu/eurobot_ws/src/eurobot_world/obstacles.world'}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot. 
    spawn_entity=Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic','robot_description',
                                 '-entity', 'mon_robot_mecanum'],
                                 output='screen')
    

    # Launch them all !
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])