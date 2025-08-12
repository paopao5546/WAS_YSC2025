import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- ส่วนที่ 1: กำหนดค่าตัวแปรและไฟล์ที่จำเป็น ---

    # หาตำแหน่งของ package ปัจจุบัน
    pkg_share = get_package_share_directory('your_package_name') # <--- !! แก้ไข 'your_package_name' เป็นชื่อ package ของคุณ

    # ระบุตำแหน่งไฟล์แผนที่ที่สร้างจาก SLAM
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml') # <--- !! แก้ไข 'maps/my_map.yaml' ถ้าจำเป็น

    # ระบุตำแหน่งไฟล์พารามิเตอร์ของ Nav2
    nav2_params_path = os.path.join(pkg_share, 'params', 'burger.yaml') # <--- !! แก้ไข 'params/burger.yaml' ถ้าจำเป็น

    # ระบุตำแหน่ง Launch file พื้นฐานของหุ่นยนต์
    robot_bringup_launch_path = os.path.join(pkg_share, 'launch', 'bring_up_new.py') # <--- !! แก้ไข 'launch/bring_up_new.py' ถ้าจำเป็น


    # --- ส่วนที่ 2: ประกาศ Launch Arguments ---

    # ประกาศให้ launch file นี้สามารถรับค่า map และ params_file จากภายนอกได้
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=map_file_path)

    param_dir = LaunchConfiguration(
        'params_file',
        default=nav2_params_path)


    # --- ส่วนที่ 3: กำหนด Node และ Launch files ที่จะรัน ---

    # 1. เรียกใช้ Launch file พื้นฐานของหุ่นยนต์ (Lidar, Motor, Odom)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch_path)
    )

    # 2. เรียกใช้ Launch file หลักของ Nav2
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    )


    # --- ส่วนที่ 4: รวมทุกอย่างเข้าด้วยกัน ---

    return LaunchDescription([
        # ประกาศ Argument ให้ ROS2 รู้จัก
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # สั่งให้รัน Launch file ทั้งสอง
        robot_bringup,
        nav2_bringup,
    ])
