# import os
# import sys
# import yaml
# from launch.actions import RegisterEventHandler, Shutdown, LogInfo
# from launch.event_handlers import OnProcessExit
# from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import Command
# sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


# def generate_launch_description():

#     from launch_ros.descriptions import ComposableNode
#     from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
#     from launch.actions import TimerAction, Shutdown
#     from launch import LaunchDescription

#     launch_params = yaml.safe_load(open(os.path.join(
#         get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

#     SetParameter(name='rune',value=launch_params['rune']),
#     robot_gimbal_description = Command(['xacro ', os.path.join(
#         get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
#         ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])
    
#     robot_navigation_description = Command(['xacro ', os.path.join(
#         get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])

#     robot_gimbal_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_gimbal_description,
#                     'publish_frequency': 1000.0}]
#     )
    
#     robot_navigation_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_navigation_description,}]
#                     # 'publish_frequency': 1000.0}]
#     )

#     def get_params(name):
#         return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))

    

#     # 图像
#     # node_params = os.path.join(
#     # get_package_share_directory('rm_bringup'), 'config', 'node_params', 'node_params.yaml')

#     # node_params = os.path.join(
#     # get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')
#     # def get_camera_node(package, plugin):
#     #     return ComposableNode(
#     #         package=package,
#     #         plugin=plugin,
#     #         name='camera_node',
#     #         parameters=[node_params],
#     #         extra_arguments=[{'use_intra_process_comms': True}]
#     #     )
#     # image_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

#     node_params = os.path.join(
#     get_package_share_directory('rm_bringup'), 'config', 'camera_params.yaml')
#     def get_camera_node(package, plugin):
#         return ComposableNode(
#             package=package,
#             plugin=plugin,
#             name='camera_node',
#             parameters=[node_params],
#             extra_arguments=[{'use_intra_process_comms': True}]
#         )
#     #image_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
#     image_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

#     # 串口
#     if launch_params['virtual_serial']:
#         serial_driver_node = Node(
#             package='rm_serial_driver',
#             executable='virtual_serial_node',
#             name='virtual_serial',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('virtual_serial')],
#             ros_arguments=['--ros-args', '-p', 'has_rune:=true' if launch_params['rune'] else 'has_rune:=false'],
#         )
#     else:
#         serial_driver_node = Node(
#             package='rm_serial_driver',
#             executable='rm_serial_driver_node',
#             name='serial_driver',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('serial_driver')],
#             ros_arguments=['--ros-args', ],
#         )
        
#     # 装甲板识别
#     armor_detector_node = ComposableNode(
#         package='openvino_armor_detector', 
#         plugin='rm_auto_aim::OpenVINODetectNode',
#         name='openvino_armor_detector',
#         extra_arguments=[{'use_intra_process_comms': True}]
#     )
    
#     # 装甲板解算
#     if launch_params['hero_solver']:
#         armor_solver_node = Node(
#             package='hero_armor_solver',
#             executable='hero_armor_solver_node',
#             name='armor_solver',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('armor_solver')],
#             ros_arguments=[],
#         )
#     else:
#         armor_solver_node = Node(
#             package='armor_solver',
#             executable='armor_solver_node',
#             name='armor_solver',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('armor_solver')],
#             ros_arguments=[],
#         )

#     # armor_tracker_node = Node(
#     #         package='armor_tracker',
#     #         executable='armor_tracker_node',
#     #         name='armor_tracker',
#     #         output='both',
#     #         emulate_tty=True,
#     #         parameters=[get_params('armor_tracker')],
#     #         ros_arguments=[],
#     #     )

#     # 打符
#     rune_detector_node = ComposableNode(    
#         package='rune_detector',
#         plugin='fyt::rune::RuneDetectorNode',
#         name='rune_detector',
#         parameters=[get_params('rune_detector')],
#         extra_arguments=[{'use_intra_process_comms': True}]
#         )
#     rune_solver_node = Node(
#         package='rune_solver',
#         executable='rune_solver_node',
#         name='rune_solver',
#         output='both',
#         emulate_tty=True,
#         parameters=[get_params('rune_solver')],
#         arguments=['--ros-args',], 
#         )

#     # 使用intra cmmunication提高图像的传输速度
#     def get_camera_detector_container(*detector_nodes):
#         nodes_list = list(detector_nodes)
#         nodes_list.append(image_node)
#         container = ComposableNodeContainer(
#             name='camera_detector_container',
#             namespace='',
#             package='rclcpp_components',
#             executable='component_container_mt',
#             composable_node_descriptions=nodes_list,
#             output='both',
#             emulate_tty=True,
#             ros_arguments=['--ros-args', ],
#         )
#         return TimerAction(
#             period=2.0,
#             actions=[container],
#         )

#     # 延迟启动
#     delay_serial_node = TimerAction(
#         period=1.5,
#         actions=[serial_driver_node],
#     )

#     delay_armor_solver_node = TimerAction(
#         period=2.0,
#         actions=[armor_solver_node],
#     )
    
#     delay_rune_solver_node = TimerAction(
#         period=2.0,
#         actions=[rune_solver_node],
#     )
    
#     if launch_params['rune']:
#         cam_detector_node = get_camera_detector_container(armor_detector_node, rune_detector_node)
#     else:
#         cam_detector_node = get_camera_detector_container(armor_detector_node)

#     delay_cam_detector_node = TimerAction(
#         period=2.0,
#         actions=[cam_detector_node],
#         ) 
    
#     push_namespace = PushRosNamespace(launch_params['namespace'])
    
#     launch_description_list = [
#         robot_gimbal_publisher,
#         push_namespace,
#         delay_serial_node,
#         delay_cam_detector_node,
#         delay_armor_solver_node]
    
#     if launch_params['rune']:
#         launch_description_list.append(delay_rune_solver_node)
    
#     if launch_params['navigation']:
#         launch_description_list.append(robot_navigation_publisher)
    
#     return LaunchDescription(launch_description_list)



# ###无container版本
# import os
# import sys
# import yaml
# from launch.actions import RegisterEventHandler, Shutdown, LogInfo
# from launch.event_handlers import OnProcessExit
# from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import Command
# sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))

# def generate_launch_description():
#     from launch_ros.actions import Node, SetParameter, PushRosNamespace
#     from launch.actions import TimerAction
#     from launch import LaunchDescription

#     launch_params = yaml.safe_load(open(os.path.join(
#         get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

#     # 初始化参数和机器人描述（保持原样）
#     SetParameter(name='rune', value=launch_params['rune']),
#     robot_gimbal_description = Command(['xacro ', os.path.join(
#         get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
#         ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])
    
#     robot_navigation_description = Command(['xacro ', os.path.join(
#         get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])

#     robot_gimbal_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_gimbal_description,
#                     'publish_frequency': 1000.0}]
#     )
    
#     robot_navigation_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_navigation_description}]
#     )

#     def get_params(name):
#         return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', f'{name}_params.yaml')

#     # ================== 修改部分开始 ================== #
#     # 摄像头节点（转换为常规节点）
#     camera_node = Node(
#         package='mindvision_camera',
#         executable='mindvision_camera_node',  # 假设可执行文件名为此
#         name='camera_node',
#         parameters=[os.path.join(
#             get_package_share_directory('rm_bringup'),
#             'config', 'camera_params.yaml')],
#         output='both',
#         emulate_tty=True,
#     )

#     # 装甲板检测节点（转换为常规节点）
#     openvino_armor_detector_node = Node(
#         package='openvino_armor_detector',
#         executable='detector_node',  # 假设可执行文件名为此
#         name='openvino_armor_detector',
#         output='both',
#         emulate_tty=True,
#     )


#     # 符石检测节点（转换为常规节点）
#     rune_detector_node = Node(
#         package='rune_detector',
#         executable='rune_detector_node',
#         name='rune_detector',
#         parameters=[get_params('rune_detector')],
#         output='both',
#         emulate_tty=True,
#     )
#     # ================== 修改部分结束 ================== #

#     # 串口节点（保持原样）
#     if launch_params['virtual_serial']:
#         serial_driver_node = Node(
#             package='rm_serial_driver',
#             executable='virtual_serial_node',
#             name='virtual_serial',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('virtual_serial')],
#             ros_arguments=['--ros-args', '-p', 'has_rune:=true' if launch_params['rune'] else 'has_rune:=false'],
#         )
#     else:
#         serial_driver_node = Node(
#             package='rm_serial_driver',
#             executable='rm_serial_driver_node',
#             name='serial_driver',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('serial_driver')],
#         )

#     # 装甲板解算节点（保持原样）
#     if launch_params['hero_solver']:
#         armor_solver_node = Node(
#             package='hero_armor_solver',
#             executable='hero_armor_solver_node',
#             name='armor_solver',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('armor_solver')],
#         )
#     else:
#         armor_solver_node = Node(
#             package='armor_solver',
#             executable='armor_solver_node',
#             name='armor_solver',
#             output='both',
#             emulate_tty=True,
#             parameters=[get_params('armor_solver')],
#         )

#     # 符石解算节点（保持原样）
#     rune_solver_node = Node(
#         package='rune_solver',
#         executable='rune_solver_node',
#         name='rune_solver',
#         output='both',
#         emulate_tty=True,
#         parameters=[get_params('rune_solver')],
#     )

#     # ================== 新的启动顺序 ================== #
#     push_namespace = PushRosNamespace(launch_params['namespace'])

#     # 延迟启动配置
#     launch_description_list = [
#         robot_gimbal_publisher,
#         push_namespace,
#         TimerAction(  # 摄像头节点最先启动
#             period=1.0,
#             actions=[camera_node]
#         ),
#         TimerAction(  # 检测节点延迟2秒
#             period=2.0,
#             actions=[openvino_armor_detector_node]
#         ),
#         TimerAction(  # 其他节点延迟3秒
#             period=3.0,
#             actions=[serial_driver_node, armor_solver_node]
#         )
#     ]

#     # 符石相关节点
#     if launch_params['rune']:
#         launch_description_list += [
#             TimerAction(
#                 period=2.5,
#                 actions=[rune_detector_node]
#             ),
#             TimerAction(
#                 period=3.5,
#                 actions=[rune_solver_node]
#             )
#         ]

#     if launch_params['navigation']:
#         launch_description_list.append(robot_navigation_publisher)

#     return LaunchDescription(launch_description_list)


import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))

def generate_launch_description():
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, Shutdown, RegisterEventHandler, LogInfo
    from launch.event_handlers import OnProcessExit
    from launch import LaunchDescription

    # 加载launch参数
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

    # 设置参数和URDF描述
    SetParameter(name='rune', value=launch_params['rune']),
    
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])
    
    robot_navigation_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])

    # 机器人状态发布节点
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

    # 参数文件加载函数
    def get_params(name):
        return os.path.join(
            get_package_share_directory('rm_bringup'), 
            'config', 
            'node_params', 
            f'{name}_params.yaml'
        )

    # 相机节点配置
    node_params = os.path.join(
        get_package_share_directory('rm_bringup'), 
        'config', 
        'camera_params.yaml'
    )
    
    camera_node = ComposableNode(
        package='hik_camera',
        plugin='hik_camera::HikCameraNode',
        name='camera_node',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 串口节点配置
    if launch_params['virtual_serial']:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='virtual_serial_node',
            name='virtual_serial',
            output='both',
            emulate_tty=True,
            parameters=[get_params('virtual_serial')],
            ros_arguments=['--ros-args', '-p', f'has_rune:={str(launch_params["rune"]).lower()}']
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

    # 装甲板检测节点
    armor_detector_node = ComposableNode(
        package='openvino_armor_detector', 
        plugin='fyt::auto_aim::OpenVINODetectNode',
        name='openvino_armor_detector',
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 装甲板解算节点
    armor_solver_node = Node(
        package='hero_armor_solver' if launch_params['hero_solver'] else 'armor_solver',
        executable='hero_armor_solver_node' if launch_params['hero_solver'] else 'armor_solver_node',
        name='armor_solver',
        output='both',
        emulate_tty=True,
        parameters=[get_params('armor_solver')],
    )

    # 符检测节点
    rune_nodes = []
    if launch_params['rune']:
        rune_detector_node = ComposableNode(    
            package='rune_detector',
            plugin='fyt::rune::RuneDetectorNode',
            name='rune_detector',
            parameters=[get_params('rune_detector')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        rune_solver_node = Node(
            package='rune_solver',
            executable='rune_solver_node',
            name='rune_solver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('rune_solver')],
        )
        rune_nodes = [rune_detector_node, rune_solver_node]

    # 创建相机检测容器（关键修改部分）
    detector_nodes = [armor_detector_node]
    if launch_params['rune']:
        detector_nodes.append(rune_detector_node)
    
    camera_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera_node] + detector_nodes,
        parameters=[
            {'on_component_shutdown': 'shutdown'}  # 组件退出时关闭容器
        ],
        output='both',
        emulate_tty=True,
    )

    # 容器退出事件处理（修正Shutdown参数）
    container_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=camera_container,
            on_exit=[
                LogInfo(msg="[紧急] 相机节点崩溃，触发系统关闭！"),
                Shutdown(reason='Camera failure detected')  # 移除exit_code参数
            ]
        )
    )

    # 延迟启动配置
    delay_config = [
        TimerAction(period=1.5, actions=[serial_driver_node]),
        TimerAction(period=2.0, actions=[camera_container]),
        TimerAction(period=2.0, actions=[armor_solver_node])
    ]
    if launch_params['rune']:
        delay_config.append(TimerAction(period=2.0, actions=[rune_solver_node]))

    # 构建launch描述
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        PushRosNamespace(launch_params['namespace']),
        robot_gimbal_publisher,
        container_exit_handler,
        *delay_config,
        *( [robot_navigation_publisher] if launch_params['navigation'] else [] )
    ])