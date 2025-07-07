import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description='Top-level namespace for RViz.')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply namespace to RViz node')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Path to the RViz config file.')

    # 기본 RViz 실행
    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen')

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={'<robot_namespace>': ('/', namespace)})

    # 네임스페이스가 적용된 RViz 실행
    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
        remappings=[
            ('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose')
        ])

    # RViz 종료 시 전체 shutdown
    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # 🔁 sub1~sub4 tf relay 노드 추가
    namespaces = ['sub1', 'sub2', 'sub3', 'sub4']
    tf_relay_nodes = []

    for ns in namespaces:
        tf_relay_nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'{ns}_tf_relay',
            arguments=[f'/{ns}/tf', '/tf'],
            output='screen'
        ))
        tf_relay_nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'{ns}_tf_static_relay',
            arguments=[f'/{ns}/tf_static', '/tf_static'],
            output='screen'
        ))

    # 전체 launch description 구성
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    for relay_node in tf_relay_nodes:
        ld.add_action(relay_node)

    return ld
