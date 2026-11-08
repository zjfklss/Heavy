import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'armor_detector' # 请确认你的功能包名称是否为 armor_detector

    # 定义参数配置
    # 1. 敌方颜色: 0-RED, 1-BLUE (调试时可以随时改)
    enemy_color_arg = DeclareLaunchArgument(
        'enemy_color', default_value='1',
        description='Enemy color: 0 for Red, 1 for Blue'
    )

    # 2. 调试模式: true 才会发布中间图像
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='true',
        description='Enable debug mode'
    )

    # 3. 数据采集模式: true 开启保存图片功能
    save_arg = DeclareLaunchArgument(
        "debug.save_dataset", default_value="true",
        description="Enable dataset saving mode"
    )

    # 获取参数值
    enemy_color = LaunchConfiguration('enemy_color')
    debug = LaunchConfiguration('debug')
    save_dataset= LaunchConfiguration('debug.save_dataset')

    # --- 节点 1: 装甲板识别 ---
    # 加载默认参数文件 (如果有的话，没有则使用代码默认值)
    # config_file = os.path.join(get_package_share_directory(package_name), 'config', 'armor_detector.yaml')
    
    detector_node = Node(
        package=package_name,
        executable='armor_detector_node', # 请确认 CMakeLists.txt 中编译的可执行文件名
        name='armor_detector',
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True,
        parameters=[{
            'debug': debug,
            'detect_color': enemy_color, # 注意：你的代码里似乎是用 'enemy_color' 读取参数
            'debug.save_dataset': save_dataset,
            'binary_thres': 80,          # 二值化阈值，根据环境光调整
            'subscribe_compressed': False, # 电脑本地测试通常不需要压缩传输
            
            # 你的代码里写死的路径，如果需要在这修改，得改代码支持传参
            # 或者在这里通过 param 覆盖 (前提是代码用 declare_parameter 读取了)
            # 'yolo_model_path': '...' 
        }],
        # 你的代码里订阅的是 /image_raw 和 /camera_info
        remappings=[
            ('/image_raw', '/image_raw'),
            ('/camera_info', '/camera_info'),
        ]
    )

    # --- 节点 3: 模拟 GameStatus 发布 (可选) ---
    # 你的代码里订阅了 /game_status 来切换相机ID，如果不发这个，
    # 可能导致 camera_status = -1，进而导致 pnp_solver 不初始化。
    # 我们用 ros2 topic pub 命令模拟一个静态发布器，或者写一个临时的 dummy node。
    # 这里为了简单，我们通过命令行手动发布，或者忽略它（如果你修改代码跳过检查）。
    # 但根据你的代码逻辑： if(pnp_solver_ == nullptr) ...
    # 必须收到一次 game_status 才能初始化 PnP。
    
    # 既然是调试，建议使用命令行手动发布一次：
    # ros2 topic pub /game_status jlcv_interfaces/msg/GameStatus "{camera_status: 1}" --once

    # --- 节点 4: RViz2 ---
    # 加载一个预设的 rviz 配置文件会方便很多
    # rviz_config = os.path.join(get_package_share_directory(package_name), 'launch', 'debug.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        enemy_color_arg,
        debug_arg,
        save_arg,
        detector_node,
        rviz_node
    ])
