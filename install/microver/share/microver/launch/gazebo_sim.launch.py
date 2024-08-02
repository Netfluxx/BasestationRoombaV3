import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    robot_xacro_name="microver"
    package_name='microver'

    robot_description_config = xacro.process_file(xacro_file)

    pkg_path = os.path.join(get_package_share_directory('microver'))
    xacro_file = os.path.join(pkg_path,'description','microver.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    description_file_relative_path = "description/microver.urdf.xacro"
    world_file_relative_path = "description/empty_world.world"
    path_description_file = os.path.join(
        get_package_share_directory(package_name), 
        description_file_relative_path
        )
    path_world_file = os.path.join(
        get_package_share_directory(package_name), 
        world_file_relative_path
        )

    
    gazebo_ros_package_launch=PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 
        'launch', 'gazebo.launch.py')
        )
    
    gazebo_launch = IncludeLaunchDescription(
        gazebo_ros_package_launch, 
        launch_arguments={'world': path_world_file}.items()
        )


    spawn_description_node = Node(package="gazebo_ros", executable="spawn_entity.py",
                                  arguments=["-topic", 'robot_description', "-entity", robot_xacro_name],
                                  output="screen")
    
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        arguments=[path_description_file]
    )

    launch_description_object = LaunchDescription()
    launch_description_object.add_action(gazebo_launch)
    launch_description_object.add_action(spawn_description_node)
    launch_description_object.add_action(node_robot_state_publisher)

    return launch_description_object