from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    set_model_env = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    empty_world_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'empty_world.launch.py'],
        shell=True,
        output='screen'
    )

    spawn_target = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'target_sphere',
                    '-file', os.path.expanduser('~/ros2_ws/src/my_world/models/target_sphere/model.sdf'),
                    '-x', '-1', '-y', '-3', '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    spawn_obstacle1 = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'obstacle1',
                    '-file', os.path.expanduser('~/ros2_ws/src/my_world/models/cylinder_obstacle/model.sdf'),
                    '-x', '1.0', '-y', '1.2', '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )

    spawn_obstacle2 = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'obstacle2',
                    '-file', os.path.expanduser('~/ros2_ws/src/my_world/models/cylinder_obstacle/model_1.sdf'),
                    '-x', '-1.7', '-y', '0.7', '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )

    spawn_obstacle3 = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'obstacle3',
                    '-file', os.path.expanduser('~/ros2_ws/src/my_world/models/cylinder_obstacle/model_2.sdf'),
                    '-x', '0.0', '-y', '-1.5', '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )

    mpc_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='mpc_follower',
                executable='mpc_follower_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_model_env,
        empty_world_launch,
        spawn_target,
        spawn_obstacle1,
        spawn_obstacle2,
        spawn_obstacle3,
        mpc_node,
    ])
