import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('attach_shelf'),
        'rviz',
        'rvizexample.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    service_node = Node(
        package = 'attach_shelf',
        executable = 'approach_service_server',
        name='approach_service_server',
        output = 'screen',
        emulate_tty = True,
        parameters=[{'use_sim_time': True}]    
    )

    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.1'
    )
    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='-10'
    )
    final_approach_arg = DeclareLaunchArgument(
        'final_approach',
        default_value='false'
    )

    pre_approach_node_v2 = Node(
        package = 'attach_shelf',
        executable = 'pre_approach_v2',
        name = 'pre_approach_v2',
        output = 'screen',
        emulate_tty = True,
        parameters=[{'use_sim_time': True,
                    'obstacle': LaunchConfiguration('obstacle'),
                    'degrees': LaunchConfiguration('degrees'),
                    'final_approach': LaunchConfiguration('final_approach')}
                   
 
        ]
    
    
    )

    return LaunchDescription([
        rviz_node,
        service_node,
        pre_approach_node_v2,
        obstacle_arg,
        degrees_arg,
        final_approach_arg
    ])