"""
Launch de simulación para VR Teleop.

Arquitectura simplificada (sin ros2_control):
  1. RSP → publica URDF, lee /joint_states, publica TFs
  2. move_group → provee /compute_ik y /compute_fk
  3. RViz → visualiza el robot
  4. vr_teleop_node → publica /joint_states directamente

No necesita ros2_control para simulación. El teleop node
publica joint states y RSP actualiza los TFs en RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cr20_moveit_dir = get_package_share_directory('cr20_moveit')
    vr_teleop_dir = get_package_share_directory('vr_teleop')

    # 1. RSP
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cr20_moveit_dir, 'launch', 'rsp.launch.py')
        ),
    )

    # 2. Static TFs (si existe)
    static_tf_path = os.path.join(
        cr20_moveit_dir, 'launch', 'static_virtual_joint_tfs.launch.py'
    )
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(static_tf_path),
    ) if os.path.exists(static_tf_path) else None

    # 3. move_group (MoveIt2 para /compute_ik y /compute_fk)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cr20_moveit_dir, 'launch', 'move_group.launch.py')
        ),
    )

    # 4. RViz
    rviz_config = os.path.join(vr_teleop_dir, 'config', 'teleop_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='log',
    )

    # 5. vr_teleop_node (delay para que move_group arranque)
    teleop_params = os.path.join(vr_teleop_dir, 'config', 'teleop_params.yaml')
    teleop_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='vr_teleop',
                executable='teleop_node',
                name='vr_teleop_node',
                parameters=[teleop_params],
                output='screen',
            ),
        ],
    )

    ld = LaunchDescription()
    if static_tf is not None:
        ld.add_action(static_tf)
    ld.add_action(rsp_launch)
    ld.add_action(move_group_launch)
    ld.add_action(rviz_node)
    ld.add_action(teleop_node)
    return ld
