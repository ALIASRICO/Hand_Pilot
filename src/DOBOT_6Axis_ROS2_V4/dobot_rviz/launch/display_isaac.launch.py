from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # USANDO TU LÓGICA ORIGINAL DE RUTAS
    name = str(os.getenv("DOBOT_TYPE", "cr20")) # Si no existe la variable, usa cr20 por defecto
    urdf_tutorial_path = get_package_share_path('dobot_rviz')
    
    # Esta es la ruta exacta que me mostraste en tu script
    default_model_path = urdf_tutorial_path / f'urdf/{name}_robot.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/urdf.rviz'

    # Argumentos de lanzamiento (Manteniendo tus nombres)
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    # Procesamiento de Xacro/URDF tal cual lo tienes tú
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # NODO 1: Robot State Publisher (Este es el que conecta Isaac con Rviz)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # NODO 2: Rviz2 (Tu visualizador)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # IMPORTANTE: He quitado joint_state_publisher porque Isaac Sim ya envía esos datos.
    
    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        rviz_node
    ])