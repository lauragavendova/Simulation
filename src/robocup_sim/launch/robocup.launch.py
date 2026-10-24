# ROS strana simulacie: popis robota, relay lidaru, RViz, ovladanie cez rqt
# a volitelne Nav2 + slam_toolbox. Spusta ho ../../../run.sh, ked Isaac
# publikuje /clock. Rucne:
#   ros2 launch robocup_sim robocup.launch.py [nav:=true planner:=teb|dwb|tr]
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import (IfCondition, LaunchConfigurationEquals,
                               LaunchConfigurationNotEquals)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('robocup_sim')
    model = PathJoinSubstitution([
        get_package_share_directory('robocup_description'), 'urdf',
        PythonExpression(["'robot_v2.urdf.xacro' if '", LaunchConfiguration('model'),
                          "' == 'v2' else 'robot.urdf.xacro'"])])
    use_sim_time = {'use_sim_time': True}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        # str: bez toho launch skusa vystup xacro parsovat ako YAML a padne, ak URDF
        # obsahuje napr. dvojbodku s medzerou v komentari
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str),
                     'use_sim_time': True}],
    )
    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        output='screen', parameters=[use_sim_time],
    )
    # /scan pre Nav2 a slam_toolbox: v1 z 2D lidaru, v2 rez point cloudu Hesai XT16
    scan_relay = Node(
        package='robocup_sim', executable='scan_no_hit_to_inf.py', output='screen',
        parameters=[use_sim_time],
        remappings=[('scan_in', '/scan_raw'), ('scan_out', '/scan')],
        condition=LaunchConfigurationEquals('model', 'v1'),
    )
    # Hesai je 0.44 m nad zemou: body od 5 cm nad zemou po 5 cm nad Hesaiom
    hesai_to_scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='hesai_to_scan', output='screen',
        parameters=[{'use_sim_time': True, 'target_frame': 'hesai_frame',
                     'min_height': -0.39, 'max_height': 0.05,
                     'angle_min': -math.pi, 'angle_max': math.pi,
                     'angle_increment': math.radians(0.18), 'scan_time': 0.1,
                     'range_min': 0.1, 'range_max': 120.0, 'use_inf': True,
                     'transform_tolerance': 0.1}],
        remappings=[('cloud_in', '/hesai/points'), ('scan', '/scan')],
        condition=LaunchConfigurationNotEquals('model', 'v1'),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'robocup.rviz')],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    steering = Node(
        package='rqt_robot_steering', executable='rqt_robot_steering', output='screen',
        parameters=[{'default_topic': '/cmd_vel'}],
        additional_env={'XDG_CONFIG_HOME': os.path.expanduser('~/.config/robocup_ws')},
        condition=IfCondition(LaunchConfiguration('steering')),
    )

    nav = LaunchConfiguration('nav')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'planner': LaunchConfiguration('planner'),
                          'cmd_vel_out': '/cmd_vel'}.items(),
        condition=IfCondition(nav),
    )
    # slam_toolbox je od Jazzy lifecycle node: po starte ho treba nakonfigurovat
    # a aktivovat, inak zostane unconfigured a nepublikuje /map ani map->odom
    slam_condition = IfCondition(PythonExpression(
        ["'", nav, "' == 'true' and '", LaunchConfiguration('slam'), "' == 'true'"]))
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[os.path.join(pkg, 'config', 'slam', 'slam_toolbox.yaml'), use_sim_time],
        condition=slam_condition,
    )
    slam_configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=slam_condition,
    )
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
            entities=[
                LogInfo(msg='[robocup] activating slam_toolbox'),
                EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                            transition_id=Transition.TRANSITION_ACTIVATE)),
            ],
        ),
        condition=slam_condition,
    )

    return LaunchDescription([
        DeclareLaunchArgument('rmw', default_value='rmw_fastrtps_cpp',
                              description='RMW, musi sediet s bridgeom v Isaacu'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw')),
        SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', ''),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        DeclareLaunchArgument('model', default_value='v2',
                              description='Model robota: v1 (Hokuyo) | v2 (Livox, kamera, Hesai)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('steering', default_value='true',
                              description='rqt_robot_steering, publikuje /cmd_vel'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Nav2 + slam_toolbox, ciel cez 2D Goal Pose v RViz'),
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Lokalny planner Nav2: dwb | teb | tr'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Pri nav:=true aj slam_toolbox (mapa pre RViz)'),
        robot_state_publisher,
        joint_state_publisher,
        scan_relay,
        hesai_to_scan,
        rviz,
        steering,
        nav2,
        slam,
        slam_activate,
        slam_configure,
    ])
