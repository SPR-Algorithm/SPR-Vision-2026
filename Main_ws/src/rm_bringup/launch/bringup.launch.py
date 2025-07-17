import os
import sys
import yaml
from launch.actions import RegisterEventHandler, Shutdown, LogInfo
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))

def generate_launch_description():
    from launch_ros.actions import Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction
    from launch_ros.descriptions import ComposableNode
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

    # 初始化参数和机器人描述（保持原样）
    SetParameter(name='rune', value=launch_params['rune']),
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])
    
    robot_navigation_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])

    robot_gimbal_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_gimbal_description,
                    'publish_frequency': 1000.0}]
    )
    
    robot_navigation_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_navigation_description}]
    )

    def get_params(name):
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', f'{name}_params.yaml')

    # ================== 修改部分开始 ================== #
    # 摄像头节点（转换为常规节点）
    if launch_params['mode']:
        node_params = os.path.join(
    get_package_share_directory('rm_bringup'), 'config', 'camera_params_hik.yaml')
    else:
        node_params = os.path.join(
    get_package_share_directory('rm_bringup'), 'config', 'camera_params_mv.yaml')
    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    if launch_params['mode']:
        image_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    else:
        image_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    # 装甲板检测节点（转换为常规节点）
    
    if launch_params['camera']:
        armor_detector_node = ComposableNode(
        package='armor_detector', 
        plugin='spr::auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[get_params('armor_detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    else:
        armor_detector_node = ComposableNode(
        package='openvino_armor_detector', 
        plugin='rm_auto_aim::OpenVINODetectNode',
        name='openvino_armor_detector',
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 符石检测节点（转换为常规节点）
    rune_detector_node = Node(
        package='rune_detector',
        executable='rune_detector_node',
        name='rune_detector',
        parameters=[get_params('rune_detector')],
        output='both',
        emulate_tty=True,
    )
    # ================== 修改部分结束 ================== #

    # 串口节点（保持原样）
    if launch_params['virtual_serial']:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='virtual_serial_node',
            name='virtual_serial',
            output='both',
            emulate_tty=True,
            parameters=[get_params('virtual_serial')],
            ros_arguments=['--ros-args', '-p', 'has_rune:=true' if launch_params['rune'] else 'has_rune:=false'],
        )
    else:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('serial_driver')],
        )

    # 装甲板解算节点（保持原样）
    if launch_params['hero_solver']:
        armor_solver_node = Node(
            package='hero_armor_solver',
            executable='hero_armor_solver_node',
            name='armor_solver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('armor_solver')],
        )
    else:
        armor_solver_node = Node(
            package='armor_solver',
            executable='armor_solver_node',
            name='armor_solver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('armor_solver')],
        )

    # 符石解算节点（保持原样）
    rune_solver_node = Node(
        package='rune_solver',
        executable='rune_solver_node',
        name='rune_solver',
        output='both',
        emulate_tty=True,
        parameters=[get_params('rune_solver')],
    )

    # ================== 新的启动顺序 ================== #
    push_namespace = PushRosNamespace(launch_params['namespace'])

    # 延迟启动配置
    launch_description_list = [
        robot_gimbal_publisher,
        push_namespace,
        TimerAction(  # 摄像头节点最先启动
            period=1.0,
            actions=[image_node]
        ),
        TimerAction(  # 检测节点延迟2秒
            period=2.0,
            actions=[armor_detector_node]
        ),
        TimerAction(  # 其他节点延迟3秒
            period=3.0,
            actions=[serial_driver_node, armor_solver_node]
        )
    ]

    # 符石相关节点
    if launch_params['rune']:
        launch_description_list += [
            TimerAction(
                period=2.5,
                actions=[rune_detector_node]
            ),
            TimerAction(
                period=3.5,
                actions=[rune_solver_node]
            )
        ]

    if launch_params['navigation']:
        launch_description_list.append(robot_navigation_publisher)

    return LaunchDescription(launch_description_list)