from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cr20_moveit_share = get_package_share_directory('cr20_moveit')
    
    # Robot State Publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cr20_moveit_share, 'launch', 'rsp.launch.py')
        )
    )
    
    # Move Group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cr20_moveit_share, 'launch', 'move_group.launch.py')
        )
    )
    
    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cr20_moveit_share, 'launch', 'moveit_rviz.launch.py')
        )
    )
    
    # Action Server (puente MoveIt -> Dobot)
    action_server = Node(
        package='dobot_moveit',
        executable='action_move_server',
        name='dobot_action_server',
        output='screen',
        env={
            'DOBOT_TYPE': os.getenv('DOBOT_TYPE', 'cr20'),
            'IP_address': os.getenv('IP_address', '192.168.5.1'),
        }
    )
        
    # Scene Manager (carga objetos de colisión)
    scene_manager = Node(
        package='dobot_moveit',
        executable='scene_manager',
        name='scene_manager',
        output='screen'
    )
    
    return LaunchDescription([
        rsp_launch,
        move_group_launch,
        rviz_launch,
        scene_manager,
        action_server,
    ])
