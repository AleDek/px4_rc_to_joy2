from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([

        ## launch microros agent
        # ExecuteProcess(
        #     cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        #     name='micro_agent_px4',  
        #     output='both',
        # ),


        ## Nodo rc_to_joy
        Node(
            package='px4_rc_to_joy2',  # Replace with your package name
            executable='rc_to_joy',  # Name of your executable in setup.py
            name='rc_to_joy',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('px4_rc_to_joy2'), 'config', 'rc_to_joy_params_h16.yaml')],  # Load parameters from the YAML file:cite[1]:cite[2]
            remappings=[
                ('/joy', '/rc_joy'),
            ]
        )
    ])
