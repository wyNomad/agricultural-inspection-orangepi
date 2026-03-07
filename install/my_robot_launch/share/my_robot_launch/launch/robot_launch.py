import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    # --- 1. 定义启动参数 ---
    move_conf = LaunchConfiguration('move')
    see_conf = LaunchConfiguration('see')

    declare_move_cmd = DeclareLaunchArgument(
        'move', default_value='true',
        description='是否开启底盘控制与里程计 (chassis_node, odom_node)'
    )

    declare_see_cmd = DeclareLaunchArgument(
        'see', default_value='true',
        description='是否开启相机节点与图像压缩 (astra_launch, republish_nodes)'
    )

    # --- 2. 视觉阵营 [SEE] (受 see 参数控制) ---
    
    lslidar_dir = get_package_share_directory('lslidar_driver')
    lslidar_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(lslidar_dir, 'launch', 'lsn10p_launch.py')
        ),
        condition=IfCondition(see_conf)
    )

    # Astra Pro 相机节点
    astra_camera_dir = get_package_share_directory('astra_camera')
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(astra_camera_dir, 'launch', 'astra_pro.launch.xml')
        ),
        condition=IfCondition(see_conf)
    )

    # # 彩色图压缩
    # color_republish_node = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='color_republish',
    #     arguments=['raw', 'compressed'],
    #     remappings=[('in', '/camera/color/image_raw'),
    #                 ('out/compressed', '/camera/color/image_raw/compressed')],
    #     condition=IfCondition(see_conf)
    # )

    # # 深度图压缩
    # depth_republish_node = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='depth_republish',
    #     arguments=['raw', 'compressedDepth'],
    #     remappings=[('in', '/camera/depth/image_raw'),
    #                 ('out/compressedDepth', '/camera/depth/image_raw/compressedDepth')],
    #     condition=IfCondition(see_conf)
    # )

    # --- 3. 底盘阵营 [MOVE] (受 move 参数控制) ---
    
    # 底盘串口桥接节点
    chassis_node = Node(
        package='chassis_driver',
        executable='serial_bridge',
        name='chassis_serial_bridge',
        output='screen',
        condition=IfCondition(move_conf)
    )

    # IMU 驱动节点
    imu_node = Node(
        package='imu_odometry',
        executable='imu_node',
        name='imu_node',
        output='screen',
        condition=IfCondition(move_conf)
    )

    # --- 2. 返回启动描述 ---
    return LaunchDescription([
        declare_move_cmd,
        declare_see_cmd,
        
        # [SEE] 组
        lslidar_launch,
        astra_launch,
        # color_republish_node,
        # depth_republish_node,
        
        # [MOVE] 组
        chassis_node,
        imu_node,
    ])
